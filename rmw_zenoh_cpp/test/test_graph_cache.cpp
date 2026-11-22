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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <zenoh.hxx>

#include "detail/graph_cache.hpp"
#include "detail/liveliness_utils.hpp"

namespace
{
rmw_zenoh_cpp::GraphCache make_test_graph_cache()
{
  z_id_t zid = {0};
  zid.id[0] = 0x42;
  return rmw_zenoh_cpp::GraphCache(zenoh::Id(zid));
}
}  // namespace

TEST(TestGraphCache, PublisherRemovalCallbackInvokedOnParseDel)
{
  auto cache = make_test_graph_cache();

  const std::string topic_name = "/test_pub_removal_topic";
  const std::size_t sub_gid_hash = 987654321;
  bool removal_callback_called = false;
  std::string removed_keyexpr;

  cache.register_publisher_removal_callback(
    topic_name,
    sub_gid_hash,
    [&](const rmw_zenoh_cpp::liveliness::Entity & entity) {
      removal_callback_called = true;
      removed_keyexpr = entity.liveliness_keyexpr();
    });

  const std::string pub_token =
    "@ros2_lv/0/session99/0/1/MP/%/%/my_test_node/%test_pub_removal_topic/"
    "std_msgs::msg::dds_::String_/RIHS01_abc123/::,10:,:,:,/backends:cpu:;cuda:meta";

  auto entity = rmw_zenoh_cpp::liveliness::Entity::make(pub_token);
  ASSERT_NE(entity, nullptr);

  // Before put, cache does not contain publisher.
  EXPECT_FALSE(cache.contains_publisher(*entity));

  // Put publisher into graph.
  cache.parse_put(pub_token);
  EXPECT_TRUE(cache.contains_publisher(*entity));
  EXPECT_FALSE(removal_callback_called);

  // Delete publisher from graph.
  cache.parse_del(pub_token);
  EXPECT_FALSE(cache.contains_publisher(*entity));
  EXPECT_TRUE(removal_callback_called);
  EXPECT_EQ(removed_keyexpr, pub_token);
}

TEST(TestGraphCache, NodeRemovalTriggersPublisherRemovalCallback)
{
  auto cache = make_test_graph_cache();

  const std::string topic_name = "/test_node_cascade_topic";
  const std::size_t sub_gid_hash = 11223344;
  bool removal_callback_called = false;

  cache.register_publisher_removal_callback(
    topic_name,
    sub_gid_hash,
    [&](const rmw_zenoh_cpp::liveliness::Entity &) {
      removal_callback_called = true;
    });

  const std::string pub_token =
    "@ros2_lv/0/session99/0/1/MP/%/%/cascade_node/%test_node_cascade_topic/"
    "std_msgs::msg::dds_::String_/RIHS01_abc123/::,10:,:,:,/backends:cpu:";
  const std::string node_token =
    "@ros2_lv/0/session99/0/0/NN/%/%/cascade_node";

  auto pub_entity = rmw_zenoh_cpp::liveliness::Entity::make(pub_token);
  ASSERT_NE(pub_entity, nullptr);

  // Add publisher.
  cache.parse_put(pub_token);
  EXPECT_TRUE(cache.contains_publisher(*pub_entity));

  // Now delete the entire node.
  cache.parse_del(node_token);

  EXPECT_FALSE(cache.contains_publisher(*pub_entity));
  EXPECT_TRUE(removal_callback_called);
}

TEST(TestGraphCache, UnregisterPublisherRemovalCallback)
{
  auto cache = make_test_graph_cache();

  const std::string topic_name = "/test_unreg_topic";
  const std::size_t sub_gid_hash = 55667788;
  bool removal_callback_called = false;

  cache.register_publisher_removal_callback(
    topic_name,
    sub_gid_hash,
    [&](const rmw_zenoh_cpp::liveliness::Entity &) {
      removal_callback_called = true;
    });

  // Unregister callbacks for this subscriber.
  cache.unregister_discovery_callbacks(sub_gid_hash);

  const std::string pub_token =
    "@ros2_lv/0/session99/0/1/MP/%/%/unreg_node/%test_unreg_topic/"
    "std_msgs::msg::dds_::String_/RIHS01_abc123/::,10:,:,:,/backends:cpu:";

  cache.parse_put(pub_token);
  cache.parse_del(pub_token);

  // Callback should NOT have been invoked.
  EXPECT_FALSE(removal_callback_called);
}

TEST(TestGraphCache, MultipleSubscribersReceivePublisherRemoval)
{
  auto cache = make_test_graph_cache();

  const std::string topic_name = "/test_multi_sub_topic";
  const std::size_t sub1_hash = 1001;
  const std::size_t sub2_hash = 1002;
  int sub1_count = 0;
  int sub2_count = 0;

  cache.register_publisher_removal_callback(
    topic_name,
    sub1_hash,
    [&](const rmw_zenoh_cpp::liveliness::Entity &) {
      ++sub1_count;
    });

  cache.register_publisher_removal_callback(
    topic_name,
    sub2_hash,
    [&](const rmw_zenoh_cpp::liveliness::Entity &) {
      ++sub2_count;
    });

  const std::string pub_token =
    "@ros2_lv/0/session99/0/1/MP/%/%/multi_node/%test_multi_sub_topic/"
    "std_msgs::msg::dds_::String_/RIHS01_abc123/::,10:,:,:,/backends:cpu:";

  cache.parse_put(pub_token);
  EXPECT_EQ(sub1_count, 0);
  EXPECT_EQ(sub2_count, 0);

  cache.parse_del(pub_token);
  EXPECT_EQ(sub1_count, 1);
  EXPECT_EQ(sub2_count, 1);
}
