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

// Every copy of one buffer-aware publication must carry the same sequence number. No GPU is
// needed: forged liveliness tokens announce a backend named `fake`, which makes the publisher
// create one endpoint per such subscriber, and plain Zenoh subscribers record each copy.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <zenoh.hxx>

#include "rmw_zenoh_cpp/rmw_zenoh.hpp"

using namespace std::chrono_literals;

namespace
{
std::vector<std::string> split(const std::string & s)
{
  std::vector<std::string> parts;
  std::stringstream stream(s);
  for (std::string part; std::getline(stream, part, '/'); ) {
    parts.push_back(part);
  }
  return parts;
}

// Token layout: @ros2_lv/<domain>/<zid>/<nid>/<id>/MS/<...>/%<topic>/<...>/backends:<list>
std::vector<std::string> subscription_tokens(zenoh::Session & session, const std::string & topic)
{
  std::vector<std::string> keys;
  zenoh::ZResult result;
  auto replies = session.liveliness_get(
    zenoh::KeyExpr("@ros2_lv/*/**"), zenoh::channels::FifoChannel(1024),
    zenoh::Session::LivelinessGetOptions::create_default(), &result);
  EXPECT_EQ(result, Z_OK);
  for (auto res = replies.recv(); std::holds_alternative<zenoh::Reply>(res);
    res = replies.recv())
  {
    const auto & reply = std::get<zenoh::Reply>(res);
    if (!reply.is_ok()) {
      continue;
    }
    const std::string key(reply.get_ok().get_keyexpr().as_string_view());
    const auto parts = split(key);
    if (parts.size() > 6 && parts[5] == "MS" &&
      std::find(parts.begin(), parts.end(), "%" + topic.substr(1)) != parts.end())
    {
      keys.push_back(key);
    }
  }
  return keys;
}

// Points a harvested token at `topic` with entity id `id` and the given backend list.
std::string forge_token(
  const std::string & harvested, const std::string & from_topic, const std::string & topic,
  const std::string & id, const std::string & backends)
{
  auto parts = split(harvested);
  parts[4] = id;
  parts.back() = "backends:" + backends;
  std::replace(parts.begin(), parts.end(), "%" + from_topic.substr(1), "%" + topic.substr(1));
  std::string key;
  for (const auto & part : parts) {
    key += (key.empty() ? "" : "/") + part;
  }
  return key;
}
}  // namespace

class TestSequenceNumber : public ::testing::Test
{
public:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
  static void TearDownTestCase() {rclcpp::shutdown();}
};

TEST_F(TestSequenceNumber, EveryCopyOfAPublicationCarriesTheSameNumber)
{
  const std::string topic = "/seq_num_u8";
  const std::string template_topic = "/tpl_seq_num_u8";
  auto pub_node = std::make_shared<rclcpp::Node>("seq_num_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("seq_num_sub");
  auto context = pub_node->get_node_base_interface()->get_rcl_node_handle()->context;
  auto session = rmw_zenoh_get_session(rcl_context_get_rmw_context(context));
  ASSERT_NE(session, nullptr);

  auto pub = pub_node->create_publisher<std_msgs::msg::UInt8MultiArray>(topic, 10);
  // A subscription on a sibling topic supplies a real token to copy; the publisher never sees it.
  auto template_sub = sub_node->create_subscription<std_msgs::msg::UInt8MultiArray>(
    template_topic, 10, [](std_msgs::msg::UInt8MultiArray::ConstSharedPtr) {});
  std::this_thread::sleep_for(2s);
  const auto keys = subscription_tokens(*session, template_topic);
  ASSERT_EQ(keys.size(), 1u) << "expected the template subscription's token";

  std::mutex mutex;
  // Key of the copy, mapped to the sequence numbers received on it, in order.
  std::map<std::string, std::vector<int64_t>> received;
  size_t without_attachment = 0;
  zenoh::ZResult result;
  auto raw_sub = session->declare_subscriber(
    zenoh::KeyExpr("*/" + topic.substr(1) + "/**"),
    [&](const zenoh::Sample & sample) {
      const auto attachment = sample.get_attachment();
      if (!attachment.has_value()) {
        std::lock_guard<std::mutex> lock(mutex);
        ++without_attachment;
        return;
      }
      zenoh::ext::Deserializer deserializer(attachment.value().get());
      const auto sequence_number = deserializer.deserialize<int64_t>();
      std::lock_guard<std::mutex> lock(mutex);
      received[std::string(sample.get_keyexpr().as_string_view())].push_back(sequence_number);
    },
    zenoh::closures::none,
    zenoh::Session::SubscriberOptions::create_default(),
    &result);
  ASSERT_EQ(result, Z_OK);

  // Two subscribers with a non-CPU backend get their own endpoint; the CPU one uses the CPU group.
  std::vector<zenoh::LivelinessToken> tokens;
  for (const auto & [id, backends] : std::vector<std::pair<std::string, std::string>>{
    {"9997", "cpu:"}, {"9998", "fake:"}, {"9999", "fake:"}})
  {
    tokens.push_back(
      session->liveliness_declare_token(
        zenoh::KeyExpr(forge_token(keys[0], template_topic, topic, id, backends)),
        zenoh::Session::LivelinessDeclarationOptions::create_default(),
        &result));
    ASSERT_EQ(result, Z_OK);
  }
  std::this_thread::sleep_for(2s);

  constexpr size_t kPublications = 10;
  std_msgs::msg::UInt8MultiArray msg;
  msg.data.assign({1, 2, 3});
  for (size_t i = 0; i < kPublications; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(100ms);
  }
  std::this_thread::sleep_for(1s);

  std::lock_guard<std::mutex> lock(mutex);
  // Positive control: an empty map would satisfy the comparison below.
  ASSERT_EQ(received.size(), 3u) << "expected the CPU-group key and two per-subscriber keys";
  const auto & first = received.begin()->second;
  EXPECT_EQ(first.size(), kPublications);
  EXPECT_EQ(without_attachment, 0u);
  for (const auto & [key, numbers] : received) {
    EXPECT_EQ(numbers, first) << "copy on " << key << " has other numbers than the first copy";
  }
  // Consecutive publications differ by one.
  for (size_t i = 1; i < first.size(); ++i) {
    EXPECT_EQ(first[i], first[i - 1] + 1) << "publication " << i << " does not follow " << i - 1;
  }
}
