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
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <zenoh.hxx>

#include "detail/buffer_endpoint_helpers.hpp"
#include "rcpputils/scope_exit.hpp"
#include "rmw/error_handling.h"
#include "rmw_zenoh_cpp/rmw_zenoh.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

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
  auto query_options = zenoh::Session::LivelinessGetOptions::create_default();
  query_options.timeout_ms = 200;
  auto replies = session.liveliness_get(
    zenoh::KeyExpr("@ros2_lv/*/**"), zenoh::channels::FifoChannel(1024),
    std::move(query_options), &result);
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

template<typename Predicate>
bool wait_until(Predicate predicate, std::chrono::seconds timeout = 10s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

std::vector<uint8_t> serialize(const std_msgs::msg::UInt8MultiArray & message)
{
  rclcpp::Serialization<std_msgs::msg::UInt8MultiArray> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  const auto & bytes = serialized.get_rcl_serialized_message();
  return {bytes.buffer, bytes.buffer + bytes.buffer_length};
}

const message_type_support_callbacks_t * original_callbacks = nullptr;

bool serialize_with_failure(const void * message, eprosima::fastcdr::Cdr & cdr)
{
  const auto & typed = *static_cast<const std_msgs::msg::UInt8MultiArray *>(message);
  return !(typed.data.size() == 1 && typed.data[0] == 250) &&
         original_callbacks->cdr_serialize(message, cdr);
}

bool serialize_endpoint_with_failure(
  const void * message, eprosima::fastcdr::Cdr & cdr,
  const rmw_topic_endpoint_info_t & endpoint,
  const rosidl_typesupport_fastrtps_cpp::BufferSerializationContext & context)
{
  const auto & typed = *static_cast<const std_msgs::msg::UInt8MultiArray *>(message);
  return !(typed.data.size() == 1 && typed.data[0] == 251) &&
         original_callbacks->cdr_serialize_with_endpoint(message, cdr, endpoint, context);
}
}  // namespace

class TestSequenceNumber : public ::testing::Test
{
public:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
  static void TearDownTestCase() {rclcpp::shutdown();}

protected:
  void check_copies(bool delayed_discovery);

  void check_serialization_failure(bool partial)
  {
    const std::string topic = "/seq_num_failure";
    const std::string template_topic = "/tpl_seq_num_failure";
    auto node = std::make_shared<rclcpp::Node>("seq_num_failure");
    const auto rcl_node = node->get_node_base_interface()->get_rcl_node_handle();
    const auto rmw_node = rcl_node_get_rmw_handle(rcl_node);
    auto session = rmw_zenoh_get_session(rcl_context_get_rmw_context(rcl_node->context));
    ASSERT_NE(session, nullptr);
    auto template_sub = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
      template_topic, 10, [](std_msgs::msg::UInt8MultiArray::ConstSharedPtr) {});
    std::vector<std::string> keys;
    ASSERT_TRUE(wait_until([&]() {
        keys = subscription_tokens(*session, template_topic);
        return keys.size() == 1;
    })) << "timed out waiting for the failure fixture's template token";

    // Copy the existing typesupport callbacks so faults need no production hooks.
    const auto dispatch =
      rosidl_typesupport_cpp::get_message_type_support_handle<std_msgs::msg::UInt8MultiArray>();
    const auto support = dispatch->func(dispatch, "rosidl_typesupport_fastrtps_cpp");
    ASSERT_NE(support, nullptr);
    original_callbacks = static_cast<const message_type_support_callbacks_t *>(support->data);
    ASSERT_NE(original_callbacks->cdr_serialize_with_endpoint, nullptr);
    auto callbacks = *original_callbacks;
    callbacks.cdr_serialize = serialize_with_failure;
    callbacks.cdr_serialize_with_endpoint = serialize_endpoint_with_failure;
    auto fault_support = *support;
    fault_support.data = &callbacks;
    const auto options = rmw_get_default_publisher_options();
    auto pub = rmw_create_publisher(
      rmw_node, &fault_support, topic.c_str(), &rmw_qos_profile_default, &options);
    ASSERT_NE(pub, nullptr);
    auto destroy_pub = rcpputils::make_scope_exit([&]() {
          EXPECT_EQ(rmw_destroy_publisher(rmw_node, pub), RMW_RET_OK);
    });

    std::mutex mutex;
    using Copies = std::map<std::string, std::vector<int64_t>>;
    std::map<std::vector<uint8_t>, Copies> received;
    size_t without_attachment = 0;
    zenoh::ZResult result;
    auto raw_sub = session->declare_subscriber(
      zenoh::KeyExpr("*/" + topic.substr(1) + "/**"),
      [&](const zenoh::Sample & sample) {
        const auto attachment = sample.get_attachment();
        std::lock_guard<std::mutex> lock(mutex);
        if (!attachment.has_value()) {
          ++without_attachment;
          return;
        }
        zenoh::ext::Deserializer deserializer(attachment.value().get());
        received[sample.get_payload().as_vector()][
          std::string(sample.get_keyexpr().as_string_view())].push_back(
          deserializer.deserialize<int64_t>());
      }, zenoh::closures::none, zenoh::Session::SubscriberOptions::create_default(), &result);
    ASSERT_EQ(result, Z_OK);
    std::vector<zenoh::LivelinessToken> tokens;
    std::string cpu_key;
    for (const auto & [id, backends] : std::vector<std::pair<std::string, std::string>>{
      {"9997", "cpu:"}, {"9998", "fake:"}})
    {
      const auto token = forge_token(keys[0], template_topic, topic, id, backends);
      if (backends == "cpu:") {
        const auto entity = rmw_zenoh_cpp::liveliness::Entity::make(token);
        ASSERT_NE(entity, nullptr);
        cpu_key = rmw_zenoh_cpp::buffer_endpoint_helpers::make_cpu_group_key(
          entity->topic_info()->topic_keyexpr_);
      }
      tokens.push_back(session->liveliness_declare_token(
          zenoh::KeyExpr(token), zenoh::Session::LivelinessDeclarationOptions::create_default(),
          &result));
      ASSERT_EQ(result, Z_OK);
    }
    std_msgs::msg::UInt8MultiArray probe;
    probe.data.assign({0});
    const auto probe_bytes = serialize(probe);
    ASSERT_TRUE(wait_until([&]() {
        EXPECT_EQ(rmw_publish(pub, &probe, nullptr), RMW_RET_OK);
        std::lock_guard<std::mutex> lock(mutex);
        const auto & copies = received[probe_bytes];
        return copies.count(cpu_key) && copies.size() == 2;
    })) << "timed out probing both failure-fixture endpoints";

    const auto publish_and_wait = [&](uint8_t value) {
        std_msgs::msg::UInt8MultiArray message;
        message.data.assign({value});
        const auto bytes = serialize(message);
        EXPECT_EQ(rmw_publish(pub, &message, nullptr), RMW_RET_OK);
        return wait_until([&]() {
                   std::lock_guard<std::mutex> lock(mutex);
                   return received[bytes].size() == 2;
        });
      };
    ASSERT_TRUE(publish_and_wait(1)) << "missing publication before injected failure";
    std_msgs::msg::UInt8MultiArray failure;
    failure.data.assign({static_cast<uint8_t>(partial ? 251 : 250)});
    EXPECT_EQ(rmw_publish(pub, &failure, nullptr), RMW_RET_ERROR);
    rmw_reset_error();
    ASSERT_TRUE(publish_and_wait(2)) << "missing publication after injected failure";
    std_msgs::msg::UInt8MultiArray before;
    before.data.assign({1});
    std_msgs::msg::UInt8MultiArray after;
    after.data.assign({2});
    int64_t last_number;
    {
      std::lock_guard<std::mutex> lock(mutex);
      const auto & first = received[serialize(before)][cpu_key];
      const auto & last = received[serialize(after)][cpu_key];
      ASSERT_EQ(first.size(), 1u);
      ASSERT_EQ(last.size(), 1u);
      last_number = last[0];
      EXPECT_EQ(last[0],
        first[0] + (partial ? 2 : 1)) << "counter advanced incorrectly after failure";
      EXPECT_EQ(without_attachment, 0u);
      if (partial) {
        const auto & escaped = received[serialize(failure)][cpu_key];
        ASSERT_EQ(escaped.size(), 1u) << "CPU copy must precede the injected endpoint failure";
        EXPECT_EQ(escaped[0], first[0] + 1);
      } else {
        EXPECT_TRUE(received[serialize(failure)].empty());
      }
    }

    // Serialized publication uses the plain base key and the same publisher counter.
    std_msgs::msg::UInt8MultiArray plain;
    plain.data.assign({3});
    const auto plain_bytes = serialize(plain);
    rclcpp::SerializedMessage serialized;
    rclcpp::Serialization<std_msgs::msg::UInt8MultiArray> serialization;
    serialization.serialize_message(&plain, &serialized);
    EXPECT_EQ(rmw_publish_serialized_message(pub, &serialized.get_rcl_serialized_message(),
      nullptr),
      RMW_RET_OK);
    ASSERT_TRUE(wait_until([&]() {
        std::lock_guard<std::mutex> lock(mutex);
        return received[plain_bytes].size() == 1;
    })) << "missing plain-path publication";
    ASSERT_TRUE(publish_and_wait(4)) << "missing buffer-aware publication after the plain path";
    std_msgs::msg::UInt8MultiArray resumed;
    resumed.data.assign({4});
    std::lock_guard<std::mutex> lock(mutex);
    const auto & plain_copies = received[plain_bytes];
    ASSERT_EQ(plain_copies.size(), 1u);
    EXPECT_EQ(plain_copies.begin()->first, cpu_key.substr(0, cpu_key.size() - 9));
    ASSERT_EQ(plain_copies.begin()->second.size(), 1u);
    EXPECT_EQ(plain_copies.begin()->second[0], last_number + 1);
    const auto & resumed_numbers = received[serialize(resumed)][cpu_key];
    ASSERT_EQ(resumed_numbers.size(), 1u);
    EXPECT_EQ(resumed_numbers[0], last_number + 2);
  }
};

TEST_F(TestSequenceNumber, FailureBeforeFirstPutDoesNotConsumeNumber)
{
  check_serialization_failure(false);
}

TEST_F(TestSequenceNumber, PartialPublicationConsumesOneNumber)
{
  check_serialization_failure(true);
}

void TestSequenceNumber::check_copies(bool delayed_discovery)
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
  std::vector<std::string> keys;
  ASSERT_TRUE(wait_until([&]() {
      keys = subscription_tokens(*session, template_topic);
      return keys.size() == 1;
  })) << "timed out waiting for the template subscription token";
  ASSERT_EQ(keys.size(), 1u) << "expected the template subscription's token";

  std::mutex mutex;
  // Key of the copy, mapped to the sequence numbers received on it, in order.
  std::map<std::string, std::vector<int64_t>> received;
  std::map<std::string, std::vector<int64_t>> timestamps;
  std::set<std::string> probed;
  std::set<std::string> expected;
  size_t without_attachment = 0;
  size_t unexpected_payload = 0;
  std_msgs::msg::UInt8MultiArray probe;
  probe.data.assign({0});
  std_msgs::msg::UInt8MultiArray msg;
  msg.data.assign({1, 2, 3});
  const auto probe_bytes = serialize(probe);
  const auto message_bytes = serialize(msg);
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
      const auto source_timestamp = deserializer.deserialize<int64_t>();
      const auto payload = sample.get_payload().as_vector();
      std::lock_guard<std::mutex> lock(mutex);
      const std::string key(sample.get_keyexpr().as_string_view());
      if (payload == probe_bytes) {
        probed.insert(key);
      } else if (payload == message_bytes) {
        received[key].push_back(sequence_number);
        timestamps[key].push_back(source_timestamp);
      } else {
        ++unexpected_payload;
      }
    },
    zenoh::closures::none,
    zenoh::Session::SubscriberOptions::create_default(),
    &result);
  ASSERT_EQ(result, Z_OK);

  // Two subscribers with a non-CPU backend get their own endpoint; the CPU one uses the CPU group.
  std::vector<zenoh::LivelinessToken> tokens;
  std::vector<std::string> token_keys;
  for (const auto & [id, backends] : std::vector<std::pair<std::string, std::string>>{
    {"9997", "cpu:"}, {"9998", "fake:"}, {"9999", "fake:"}})
  {
    const auto token = forge_token(keys[0], template_topic, topic, id, backends);
    const auto entity = rmw_zenoh_cpp::liveliness::Entity::make(token);
    ASSERT_NE(entity, nullptr);
    const auto & base = entity->topic_info()->topic_keyexpr_;
    expected.insert(
      backends == "cpu:" ? rmw_zenoh_cpp::buffer_endpoint_helpers::make_cpu_group_key(base) :
      rmw_zenoh_cpp::buffer_endpoint_helpers::make_accelerated_key(
        base, rmw_zenoh_cpp::buffer_endpoint_helpers::gid_array_to_hex(entity->copy_gid())));
    token_keys.push_back(token);
  }
  ASSERT_EQ(expected.size(), 3u);
  std::thread discovery([&]() {
      if (delayed_discovery) {
        std::this_thread::sleep_for(250ms);
      }
      for (const auto & token : token_keys) {
        zenoh::ZResult declaration_result;
        tokens.push_back(session->liveliness_declare_token(
            zenoh::KeyExpr(token), zenoh::Session::LivelinessDeclarationOptions::create_default(),
            &declaration_result));
        EXPECT_EQ(declaration_result, Z_OK);
      }
    });
  auto join_discovery = rcpputils::make_scope_exit([&]() {discovery.join();});
  ASSERT_TRUE(wait_until([&]() {
      pub->publish(probe);
      std::lock_guard<std::mutex> lock(mutex);
      return std::all_of(expected.begin(), expected.end(), [&](const auto & key) {
               return probed.count(key) != 0;
    });
  })) << "timed out probing the CPU-group and two subscriber endpoints";

  constexpr size_t kPublications = 10;
  for (size_t i = 0; i < kPublications; ++i) {
    pub->publish(msg);
    ASSERT_TRUE(wait_until([&]() {
        std::lock_guard<std::mutex> lock(mutex);
        return std::all_of(expected.begin(), expected.end(), [&](const auto & key) {
                 const auto found = received.find(key);
                 return found != received.end() && found->second.size() >= i + 1;
      });
    })) << "timed out receiving publication " << i << " on three endpoints";
  }

  std::lock_guard<std::mutex> lock(mutex);
  // Positive control: an empty map would satisfy the comparison below.
  ASSERT_EQ(received.size(), 3u) << "expected the CPU-group key and two per-subscriber keys";
  ASSERT_EQ(timestamps.size(), 3u);
  const auto & first = received.begin()->second;
  const auto & first_timestamps = timestamps.begin()->second;
  EXPECT_EQ(first.size(), kPublications);
  EXPECT_EQ(without_attachment, 0u);
  EXPECT_EQ(unexpected_payload, 0u);
  for (const auto & [key, numbers] : received) {
    EXPECT_EQ(expected.count(key), 1u) << "unexpected endpoint " << key;
    EXPECT_EQ(numbers.size(), kPublications) << "incomplete endpoint " << key;
    EXPECT_EQ(numbers, first) << "copy on " << key << " has other numbers than the first copy";
    EXPECT_EQ(timestamps.at(key), first_timestamps) << "copy on " << key <<
      " has other timestamps than the first copy";
  }
  // Consecutive publications differ by one.
  for (size_t i = 1; i < first.size(); ++i) {
    ASSERT_EQ(first[i], first[i - 1] + 1) << "publication " << i << " does not follow " << i - 1;
  }
}

TEST_F(TestSequenceNumber, EveryCopyOfAPublicationCarriesTheSameNumber)
{
  check_copies(false);
}

TEST_F(TestSequenceNumber, DelayedDiscoveryDoesNotEnterMeasuredBatch)
{
  check_copies(true);
}
