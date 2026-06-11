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

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// This test references issue ticket number 921:
// "potential deadlock between waitset condition_mutex and SubscriptionData mutex"
// It simulates heavy traffic by publishing messages concurrently with wait execution.
class TestIssue921 : public ::testing::Test
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

TEST_F(TestIssue921, TestDeadlockUnderHeavyTraffic)
{
  auto node = std::make_shared<rclcpp::Node>("test_issue_921_node");
  auto publisher = node->create_publisher<std_msgs::msg::String>("test_issue_921_topic", 10);
  
  std::atomic<size_t> received_count{0};
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "test_issue_921_topic", 10,
    [&received_count](std_msgs::msg::String::ConstSharedPtr) {
      received_count.fetch_add(1);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::atomic<bool> running{true};
  std::promise<void> test_finished_promise;
  auto test_finished_future = test_finished_promise.get_future();

  // Run the test in a thread so we can monitor for deadlocks with a watchdog
  std::thread test_thread([&]() {
    // Publisher thread that publishes messages as fast as possible
    std::thread pub_thread([&]() {
      std_msgs::msg::String msg;
      msg.data = "hello";
      for (size_t i = 0; i < 500 && running; ++i) {
        publisher->publish(msg);
        std::this_thread::sleep_for(std::chrono::microseconds(500));
      }
      running = false;
    });

    // Executor spin loop
    while (running) {
      executor.spin_some(std::chrono::milliseconds(5));
    }

    pub_thread.join();
    test_finished_promise.set_value();
  });

  // Watchdog timeout (5 seconds)
  if (test_finished_future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
    running = false;
    // We timed out! Fails the test due to deadlock
    FAIL() << "Test timed out! Possible deadlock in rmw_zenoh_cpp (Issue #921).";
    // We terminate to avoid hanging the process if threads are locked
    std::terminate();
  } else {
    test_thread.join();
  }

  // Ensure we actually received some messages
  EXPECT_GT(received_count.load(), 0u);
}
