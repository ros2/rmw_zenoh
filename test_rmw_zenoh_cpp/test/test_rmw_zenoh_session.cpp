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

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
