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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
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
  auto options = zenoh::Session::LivelinessGetOptions::create_default();
  options.timeout_ms = 200;
  auto replies = session.liveliness_get(
    zenoh::KeyExpr("@ros2_lv/*/**"),
    zenoh::channels::FifoChannel(1024),
    std::move(options),
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

template<typename Predicate>
bool wait_until(Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  do {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

// Copy @ros2_lv/<domain>/<zid>/<nid>/<id>/MS/.../<topic>/...[/backends:<list>].
std::string legacy_token(
  const std::string & source, const std::string & from_topic, const std::string & topic)
{
  std::vector<std::string> parts;
  std::stringstream stream(source);
  for (std::string part; std::getline(stream, part, '/'); ) {
    parts.push_back(part);
  }
  if (parts.size() < 10 || parts[5] != "MS") {
    ADD_FAILURE() << "unexpected subscription token format: " << source;
    return {};
  }
  parts[4] = "9999";
  if (parts.back().rfind("backends:", 0) == 0) {
    parts.pop_back();
  }
  bool replaced = false;
  for (auto & part : parts) {
    if (part == "%" + from_topic.substr(1)) {
      part = "%" + topic.substr(1);
      replaced = true;
    }
  }
  if (!replaced) {
    ADD_FAILURE() << "template topic missing from token: " << source;
    return {};
  }
  std::string token;
  for (const auto & part : parts) {
    token += (token.empty() ? "" : "/") + part;
  }
  return token;
}

// Returns the number of samples that reached the topic's base key.
// `expect_backends` is true when the topic carries a buffer field, in which
// case the harvested token must announce `backends:` before it is stripped.
template<typename MsgT>
size_t base_key_samples(
  const std::string & topic, const MsgT & msg, bool expect_backends,
  bool buffer_aware_subscriber_leaves_first = false)
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
  if (buffer_aware_subscriber_leaves_first) {
    // The publisher must forget this subscriber once it is gone.
    auto leaver = sub_node->create_subscription<MsgT>(
      topic, 10, [](typename MsgT::ConstSharedPtr) {});
    EXPECT_TRUE(wait_until([&]() {return pub->get_subscription_count() == 1;}))
      << "timed out waiting for the buffer-aware subscriber to join";
    std::atomic<bool> saw_buffer_sample{false};
    zenoh::ZResult probe_result;
    auto probe = session->declare_subscriber(
      zenoh::KeyExpr("*/" + topic.substr(1) + "/**"),
      [&](const zenoh::Sample & sample) {
        if (sample.get_keyexpr().as_string_view().find("/_buf") != std::string_view::npos) {
          saw_buffer_sample.store(true);
        }
      }, zenoh::closures::none, zenoh::Session::SubscriberOptions::create_default(),
      &probe_result);
    EXPECT_EQ(probe_result, Z_OK);
    EXPECT_TRUE(wait_until([&]() {
        pub->publish(msg);
        return saw_buffer_sample.load();
    })) << "buffer-aware subscriber did not establish the buffer path before removal";
    leaver.reset();
    EXPECT_TRUE(wait_until([&]() {return pub->get_subscription_count() == 0;}))
      << "timed out waiting for the buffer-aware subscriber to leave";
  }
  // The token is harvested from a subscription on a sibling topic, which stays
  // alive for the whole test. The publisher never sees it, and it never leaves
  // the publisher's list of discovered subscribers.
  const std::string template_topic = "/tpl_" + topic.substr(1);
  auto template_sub = sub_node->create_subscription<MsgT>(
    template_topic, 10, [](typename MsgT::ConstSharedPtr) {});
  std::vector<std::string> keys;
  EXPECT_TRUE(wait_until([&]() {
      keys = liveliness_keys(*session, template_topic);
      return keys.size() == 1;
  })) << "timed out waiting for the template subscription token";
  EXPECT_EQ(keys.size(), 1u) << "expected the template subscription's token";
  if (keys.size() != 1u) {
    return 0;
  }
  const auto backends_pos = keys[0].rfind("/backends:");
  EXPECT_EQ(backends_pos != std::string::npos, expect_backends)
    << "token of the template subscription: " << keys[0];
  const auto legacy_key = legacy_token(keys[0], template_topic, topic);
  if (legacy_key.empty()) {
    return 0;
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
  EXPECT_TRUE(wait_until([&]() {return pub->get_subscription_count() == 1;}))
    << "timed out waiting for the legacy subscriber to match";
  wait_until([&]() {
      pub->publish(msg);
      std::lock_guard<std::mutex> lock(mutex);
      return std::any_of(received_keys.begin(), received_keys.end(), [](const auto & key) {
               return key.find("/_buf") == std::string::npos;
    });
  });

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_FALSE(received_keys.empty()) << "no sample arrived on any route after publication";
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

// A subscriber that has left must not keep the publisher on the buffer path.
TEST_F(TestLegacySubscriber, BufferTopicReachesBaseKeyAfterBufferAwareSubscriberLeaves)
{
  std_msgs::msg::UInt8MultiArray msg;
  msg.data.assign({1, 2, 3});
  EXPECT_GT(base_key_samples("/legacy_sub_u8_left", msg, true, true), 0u);
}

TEST_F(TestLegacySubscriber, BufferPathReturnsAfterLegacySubscriberLeaves)
{
  const std::string topic = "/legacy_sub_u8_return";
  const std::string template_topic = "/tpl_legacy_sub_u8_return";
  auto node = std::make_shared<rclcpp::Node>("legacy_path_returns");
  auto pub = node->create_publisher<std_msgs::msg::UInt8MultiArray>(topic, 10);
  auto modern = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
    topic, 10, [](std_msgs::msg::UInt8MultiArray::ConstSharedPtr) {});
  auto template_sub = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
    template_topic, 10, [](std_msgs::msg::UInt8MultiArray::ConstSharedPtr) {});
  auto context = node->get_node_base_interface()->get_rcl_node_handle()->context;
  auto session = rmw_zenoh_get_session(rcl_context_get_rmw_context(context));
  ASSERT_NE(session, nullptr);
  std::vector<std::string> keys;
  ASSERT_TRUE(wait_until([&]() {
      keys = liveliness_keys(*session, template_topic);
      return keys.size() == 1;
  })) << "timed out waiting for the redeclaration fixture's template token";

  std::mutex mutex;
  std::vector<std::pair<std::vector<uint8_t>, std::string>> samples;
  zenoh::ZResult result;
  auto raw_sub = session->declare_subscriber(
    zenoh::KeyExpr("*/" + topic.substr(1) + "/**"),
    [&](const zenoh::Sample & sample) {
      std::lock_guard<std::mutex> lock(mutex);
      samples.emplace_back(sample.get_payload().as_vector(), sample.get_keyexpr().as_string_view());
    }, zenoh::closures::none, zenoh::Session::SubscriberOptions::create_default(), &result);
  ASSERT_EQ(result, Z_OK);
  const auto receives = [&](uint8_t marker, bool buffer_path) {
      std_msgs::msg::UInt8MultiArray message;
      message.data.assign({marker});
      rclcpp::Serialization<std_msgs::msg::UInt8MultiArray> serialization;
      rclcpp::SerializedMessage serialized;
      serialization.serialize_message(&message, &serialized);
      const auto & bytes = serialized.get_rcl_serialized_message();
      const std::vector<uint8_t> payload(bytes.buffer, bytes.buffer + bytes.buffer_length);
      return wait_until([&]() {
                 pub->publish(message);
                 std::lock_guard<std::mutex> lock(mutex);
                 return std::any_of(samples.begin(), samples.end(), [&](const auto & sample) {
                          return sample.first == payload &&
                                 (sample.second.find("/_buf") != std::string::npos) == buffer_path;
        });
      });
    };
  ASSERT_TRUE(wait_until([&]() {return pub->get_subscription_count() == 1;}))
    << "timed out waiting for the modern subscriber";
  ASSERT_TRUE(receives(11, true)) << "modern subscriber did not establish the buffer path";
  std::optional<zenoh::LivelinessToken> legacy;
  legacy.emplace(session->liveliness_declare_token(
      zenoh::KeyExpr(legacy_token(keys[0], template_topic, topic)),
      zenoh::Session::LivelinessDeclarationOptions::create_default(), &result));
  ASSERT_EQ(result, Z_OK);
  ASSERT_TRUE(wait_until([&]() {return pub->get_subscription_count() == 2;}))
    << "timed out waiting for the legacy subscriber to join";
  ASSERT_TRUE(receives(22, false)) << "legacy join did not select the base key";
  legacy.reset();
  ASSERT_TRUE(wait_until([&]() {return pub->get_subscription_count() == 1;}))
    << "timed out waiting for the legacy subscriber to leave";
  ASSERT_TRUE(receives(33, true)) << "legacy removal did not restore the buffer path";
}
