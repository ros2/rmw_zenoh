// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// A subscriber built before buffer support announces a liveliness token with
// no `backends:` field and listens only on the topic's base key. A publisher
// of a message with a buffer field must keep writing to the base key for it.
//
// The legacy subscriber is emulated: a real subscription supplies a token, the
// test removes the `backends:` field and declares the result itself, then
// listens on the topic with a plain Zenoh subscriber.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <zenoh.hxx>

#include "rmw_zenoh_cpp/rmw_zenoh.hpp"

using namespace std::chrono_literals;

namespace
{
std::vector<std::string> liveliness_keys(zenoh::Session & session, const std::string & topic)
{
  std::vector<std::string> keys;
  zenoh::ZResult result;
  auto replies = session.liveliness_get(
    zenoh::KeyExpr("@ros2_lv/*/**"),
    zenoh::channels::FifoChannel(1024),
    zenoh::Session::LivelinessGetOptions::create_default(),
    &result);
  if (result != Z_OK) {
    ADD_FAILURE() << "liveliness_get failed, result=" << static_cast<int>(result);
    return keys;
  }
  for (auto res = replies.recv(); std::holds_alternative<zenoh::Reply>(res);
    res = replies.recv())
  {
    const auto & reply = std::get<zenoh::Reply>(res);
    if (!reply.is_ok()) {
      continue;
    }
    const std::string key(reply.get_ok().get_keyexpr().as_string_view());
    // Topic names are escaped in tokens: "/chatter" becomes "%chatter".
    const std::string escaped_topic = "/%" + topic.substr(1) + "/";
    if (key.find("/MS/") != std::string::npos && key.find(escaped_topic) != std::string::npos) {
      keys.push_back(key);
    }
  }
  return keys;
}

// Returns the number of samples that reached the topic's base key.
// `expect_backends` is true when the topic carries a buffer field, in which
// case the harvested token must announce `backends:` before it is stripped.
template<typename MsgT>
size_t base_key_samples(const std::string & topic, const MsgT & msg, bool expect_backends)
{
  auto pub_node = std::make_shared<rclcpp::Node>("legacy_pub_" + topic.substr(1));
  auto sub_node = std::make_shared<rclcpp::Node>("legacy_sub_" + topic.substr(1));
  auto context = pub_node->get_node_base_interface()->get_rcl_node_handle()->context;
  auto session = rmw_zenoh_get_session(rcl_context_get_rmw_context(context));
  EXPECT_NE(session, nullptr);
  if (!session) {
    return 0;
  }

  auto pub = pub_node->create_publisher<MsgT>(topic, 10);
  // The token is harvested from a subscription on a sibling topic, which stays
  // alive for the whole test. The publisher never sees it, and it never leaves
  // the publisher's list of discovered subscribers.
  const std::string template_topic = "/tpl_" + topic.substr(1);
  auto template_sub = sub_node->create_subscription<MsgT>(
    template_topic, 10, [](typename MsgT::ConstSharedPtr) {});
  std::this_thread::sleep_for(2s);

  const auto keys = liveliness_keys(*session, template_topic);
  EXPECT_EQ(keys.size(), 1u) << "expected the template subscription's token";
  if (keys.size() != 1u) {
    return 0;
  }
  std::string legacy_key = keys[0];
  const auto backends_pos = legacy_key.rfind("/backends:");
  EXPECT_EQ(backends_pos != std::string::npos, expect_backends)
    << "token of the template subscription: " << keys[0];
  if (backends_pos != std::string::npos) {
    legacy_key.erase(backends_pos);
  }
  // Point the token at the publisher's topic and give it a fresh entity id, so
  // it is a distinct subscriber from the template one.
  // Token layout: @ros2_lv/<domain>/<zid>/<nid>/<id>/MS/...
  {
    const std::string from = "/%" + template_topic.substr(1) + "/";
    const auto topic_pos = legacy_key.find(from);
    EXPECT_NE(topic_pos, std::string::npos);
    if (topic_pos == std::string::npos) {
      return 0;
    }
    legacy_key.replace(topic_pos, from.size(), "/%" + topic.substr(1) + "/");
    size_t start = 0;
    for (int chunk = 0; chunk < 4; ++chunk) {
      start = legacy_key.find('/', start) + 1;
    }
    const size_t end = legacy_key.find('/', start);
    legacy_key.replace(start, end - start, "9999");
  }

  std::mutex mutex;
  std::vector<std::string> received_keys;
  zenoh::ZResult result;
  auto raw_sub = session->declare_subscriber(
    zenoh::KeyExpr("*/" + topic.substr(1) + "/**"),
    [&](const zenoh::Sample & sample) {
      std::lock_guard<std::mutex> lock(mutex);
      received_keys.emplace_back(sample.get_keyexpr().as_string_view());
    },
    zenoh::closures::none,
    zenoh::Session::SubscriberOptions::create_default(),
    &result);
  EXPECT_EQ(result, Z_OK);
  auto token = session->liveliness_declare_token(
    zenoh::KeyExpr(legacy_key),
    zenoh::Session::LivelinessDeclarationOptions::create_default(),
    &result);
  EXPECT_EQ(result, Z_OK);
  std::this_thread::sleep_for(2s);

  for (int i = 0; i < 40; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(100ms);
  }

  std::lock_guard<std::mutex> lock(mutex);
  size_t on_base_key = 0;
  for (const auto & key : received_keys) {
    if (key.find("/_buf") == std::string::npos) {
      ++on_base_key;
    }
  }
  return on_base_key;
}
}  // namespace

class TestLegacySubscriber : public ::testing::Test
{
public:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
  static void TearDownTestCase() {rclcpp::shutdown();}
};

// Positive control: without a buffer field the base key is always used, so this
// passes with or without the fix and shows the harness can see samples.
TEST_F(TestLegacySubscriber, NonBufferTopicReachesBaseKey)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {1.0, 2.0, 3.0};
  EXPECT_GT(base_key_samples("/legacy_sub_f64", msg, false), 0u);
}

TEST_F(TestLegacySubscriber, BufferTopicReachesBaseKeyForSubscriberWithoutBackends)
{
  std_msgs::msg::UInt8MultiArray msg;
  msg.data.assign({1, 2, 3});
  EXPECT_GT(base_key_samples("/legacy_sub_u8", msg, true), 0u);
}
