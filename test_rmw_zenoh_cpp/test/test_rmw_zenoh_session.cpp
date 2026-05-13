// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include <rcl/node.h>
#include <rcutils/allocator.h>
#include <rmw/get_node_info_and_types.h>
#include <rmw/get_service_endpoint_info.h>
#include <rmw/get_service_names_and_types.h>
#include <rmw/get_topic_endpoint_info.h>
#include <rmw/get_topic_names_and_types.h>
#include <rmw/names_and_types.h>
#include <rmw/service_endpoint_info_array.h>
#include <rmw/topic_endpoint_info_array.h>
#include <rosidl_runtime_c/type_hash.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <example_interfaces/srv/add_two_ints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <zenoh.hxx>

#include "rmw_zenoh_cpp/rmw_zenoh.hpp"

class TestRmwZenohSession : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

namespace
{
std::string
normalize_name(const std::string & name)
{
  if (!name.empty() && name.front() == '/') {
    return name.substr(1);
  }
  return name;
}

bool
is_zero_hash(const rosidl_type_hash_t & hash)
{
  if (hash.version != 0) {
    return false;
  }
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; ++i) {
    if (hash.value[i] != 0) {
      return false;
    }
  }
  return true;
}

bool
names_and_types_has_any_non_zero_hash(const rmw_names_and_types_t & nat)
{
  if (!nat.type_hashes) {
    return false;
  }
  for (size_t i = 0; i < nat.names.size; ++i) {
    if (!nat.type_hashes[i]) {
      continue;
    }
    for (size_t j = 0; j < nat.types[i].size; ++j) {
      if (!is_zero_hash(nat.type_hashes[i][j])) {
        return true;
      }
    }
  }
  return false;
}

bool
endpoint_info_has_non_zero_hash(const rmw_topic_endpoint_info_array_t & arr)
{
  for (size_t i = 0; i < arr.size; ++i) {
    if (!is_zero_hash(arr.info_array[i].topic_type_hash)) {
      return true;
    }
  }
  return false;
}

bool
service_endpoint_info_has_non_zero_hash(const rmw_service_endpoint_info_array_t & arr)
{
  for (size_t i = 0; i < arr.size; ++i) {
    if (!is_zero_hash(arr.info_array[i].service_type_hash)) {
      return true;
    }
  }
  return false;
}

bool
contains_non_zero_hash_for_type(
  const rmw_names_and_types_t & names_and_types,
  const std::string & expected_name,
  const std::string & expected_type)
{
  if (!names_and_types.type_hashes) {
    return false;
  }

  const std::string normalized_expected_name = normalize_name(expected_name);
  for (size_t i = 0; i < names_and_types.names.size; ++i) {
    if (normalize_name(names_and_types.names.data[i]) != normalized_expected_name) {
      continue;
    }
    if (!names_and_types.type_hashes[i]) {
      continue;
    }
    for (size_t j = 0; j < names_and_types.types[i].size; ++j) {
      if (expected_type != names_and_types.types[i].data[j]) {
        continue;
      }
      if (!is_zero_hash(names_and_types.type_hashes[i][j])) {
        return true;
      }
    }
  }
  return false;
}
}  // namespace

TEST_F(TestRmwZenohSession, GetZenohSessionFromContext)
{
  // Create a node
  auto node = std::make_shared<rclcpp::Node>("test_zenoh_session");

  // Get the RMW context from rclcpp
  auto node_handle = node->get_node_base_interface()->get_shared_rcl_node_handle();
  auto rcl_node = node_handle.get();
  auto context = rcl_node->context;

  // Get the RMW context
  auto rmw_context = rcl_context_get_rmw_context(context);
  ASSERT_NE(rmw_context, nullptr);

  // Get the Zenoh session
  auto session = rmw_zenoh_get_session(rmw_context);
  ASSERT_NE(session, nullptr) << "Failed to get Zenoh session from RMW context";
}

TEST_F(TestRmwZenohSession, ZenohSessionDirectAccess)
{
  // Create a node
  auto node = std::make_shared<rclcpp::Node>("test_zenoh_session_direct");

  // Get the RMW context
  auto node_handle = node->get_node_base_interface()->get_shared_rcl_node_handle();
  auto rcl_node = node_handle.get();
  auto context = rcl_node->context;
  auto rmw_context = rcl_context_get_rmw_context(context);

  // Get the Zenoh session
  auto session = rmw_zenoh_get_session(rmw_context);
  ASSERT_NE(session, nullptr);

  // Verify we can get the ZID (Zenoh ID) from the session
  auto zid = session->get_zid();
  // ZID is a 16-byte array, just verify it exists
  (void)zid;  // Suppress unused variable warning

  // Create a simple keyexpr to test session functionality
  const std::string test_key = "test/rmw_zenoh/session";
  zenoh::KeyExpr keyexpr(test_key);

  // Test creating a publisher using the session directly
  // This demonstrates that applications can use the session for advanced Zenoh operations
  auto publisher = session->declare_publisher(keyexpr);

  // Test publishing data
  const std::string test_data = "test_message";
  publisher.put(test_data);

  // Give some time for the operation to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST_F(TestRmwZenohSession, NamesAndTypesPopulateTypeHashes)
{
  auto pub_node = std::make_shared<rclcpp::Node>("hash_writer");
  auto sub_node = std::make_shared<rclcpp::Node>("hash_reader");
  auto query_node = std::make_shared<rclcpp::Node>("hash_query");

  auto pub = pub_node->create_publisher<std_msgs::msg::String>("hash_topic", 10);
  auto sub = sub_node->create_subscription<std_msgs::msg::String>(
    "hash_topic", 10, [](const std_msgs::msg::String::SharedPtr) {});
  (void)pub;
  (void)sub;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);
  executor.add_node(query_node);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  const rcl_node_t * rcl_node = query_node->get_node_base_interface()->get_rcl_node_handle();
  const rmw_node_t * rmw_node = rcl_node_get_rmw_handle(rcl_node);
  ASSERT_NE(nullptr, rmw_node);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  rmw_names_and_types_t topic_names_and_types = rmw_get_zero_initialized_names_and_types();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_topic_names_and_types(
      rmw_node,
      &allocator,
      false,
      &topic_names_and_types));

  EXPECT_TRUE(contains_non_zero_hash_for_type(
      topic_names_and_types,
      "/hash_topic",
      "std_msgs/msg/String"));
  ASSERT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&topic_names_and_types));

  rmw_names_and_types_t publisher_names_and_types = rmw_get_zero_initialized_names_and_types();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_publisher_names_and_types_by_node(
      rmw_node,
      &allocator,
      "hash_writer",
      "/",
      false,
      &publisher_names_and_types));

  EXPECT_TRUE(contains_non_zero_hash_for_type(
      publisher_names_and_types,
      "/hash_topic",
      "std_msgs/msg/String"));
  ASSERT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&publisher_names_and_types));

  rmw_names_and_types_t subscriber_names_and_types = rmw_get_zero_initialized_names_and_types();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_subscriber_names_and_types_by_node(
      rmw_node,
      &allocator,
      "hash_reader",
      "/",
      false,
      &subscriber_names_and_types));
  EXPECT_TRUE(contains_non_zero_hash_for_type(
      subscriber_names_and_types,
      "/hash_topic",
      "std_msgs/msg/String"));
  ASSERT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&subscriber_names_and_types));

  rmw_topic_endpoint_info_array_t pubs_info =
    rmw_get_zero_initialized_topic_endpoint_info_array();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_publishers_info_by_topic(
      rmw_node, &allocator, "/hash_topic", false, &pubs_info));
  EXPECT_TRUE(endpoint_info_has_non_zero_hash(pubs_info));
  ASSERT_EQ(RMW_RET_OK, rmw_topic_endpoint_info_array_fini(&pubs_info, &allocator));

  rmw_topic_endpoint_info_array_t subs_info =
    rmw_get_zero_initialized_topic_endpoint_info_array();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_subscriptions_info_by_topic(
      rmw_node, &allocator, "/hash_topic", false, &subs_info));
  EXPECT_TRUE(endpoint_info_has_non_zero_hash(subs_info));
  ASSERT_EQ(RMW_RET_OK, rmw_topic_endpoint_info_array_fini(&subs_info, &allocator));

  executor.remove_node(pub_node);
  executor.remove_node(sub_node);
  executor.remove_node(query_node);
}

TEST_F(TestRmwZenohSession, ServiceTypeHashesPopulated)
{
  auto srv_node = std::make_shared<rclcpp::Node>("hash_server");
  auto cli_node = std::make_shared<rclcpp::Node>("hash_client");
  auto query_node = std::make_shared<rclcpp::Node>("hash_svc_query");

  auto srv = srv_node->create_service<example_interfaces::srv::AddTwoInts>(
    "hash_service",
    [](
      const example_interfaces::srv::AddTwoInts::Request::SharedPtr,
      example_interfaces::srv::AddTwoInts::Response::SharedPtr) {});
  auto cli = cli_node->create_client<example_interfaces::srv::AddTwoInts>("hash_service");
  (void)srv;
  (void)cli;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(srv_node);
  executor.add_node(cli_node);
  executor.add_node(query_node);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  const rcl_node_t * rcl_node = query_node->get_node_base_interface()->get_rcl_node_handle();
  const rmw_node_t * rmw_node = rcl_node_get_rmw_handle(rcl_node);
  ASSERT_NE(nullptr, rmw_node);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  rmw_names_and_types_t svc_nat = rmw_get_zero_initialized_names_and_types();
  ASSERT_EQ(RMW_RET_OK, rmw_get_service_names_and_types(rmw_node, &allocator, &svc_nat));
  EXPECT_TRUE(names_and_types_has_any_non_zero_hash(svc_nat));
  ASSERT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&svc_nat));

  rmw_names_and_types_t srv_nat = rmw_get_zero_initialized_names_and_types();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_service_names_and_types_by_node(
      rmw_node, &allocator, "hash_server", "/", &srv_nat));
  EXPECT_TRUE(names_and_types_has_any_non_zero_hash(srv_nat));
  ASSERT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&srv_nat));

  rmw_names_and_types_t cli_nat = rmw_get_zero_initialized_names_and_types();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_client_names_and_types_by_node(
      rmw_node, &allocator, "hash_client", "/", &cli_nat));
  EXPECT_TRUE(names_and_types_has_any_non_zero_hash(cli_nat));
  ASSERT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&cli_nat));

  rmw_service_endpoint_info_array_t servers_info =
    rmw_get_zero_initialized_service_endpoint_info_array();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_servers_info_by_service(
      rmw_node, &allocator, "/hash_service", false, &servers_info));
  EXPECT_TRUE(service_endpoint_info_has_non_zero_hash(servers_info));
  ASSERT_EQ(RMW_RET_OK, rmw_service_endpoint_info_array_fini(&servers_info, &allocator));

  rmw_service_endpoint_info_array_t clients_info =
    rmw_get_zero_initialized_service_endpoint_info_array();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_clients_info_by_service(
      rmw_node, &allocator, "/hash_service", false, &clients_info));
  EXPECT_TRUE(service_endpoint_info_has_non_zero_hash(clients_info));
  ASSERT_EQ(RMW_RET_OK, rmw_service_endpoint_info_array_fini(&clients_info, &allocator));

  executor.remove_node(srv_node);
  executor.remove_node(cli_node);
  executor.remove_node(query_node);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
