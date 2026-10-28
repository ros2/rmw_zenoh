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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/scope_exit.hpp>
#include <test_msgs/msg/unbounded_sequences.hpp>

using namespace std::chrono_literals;

// Regression test: publishing a message type with a uint8[] field while
// rclcpp::shutdown() closes the zenoh session must not throw.
TEST(TestPublishDuringShutdown, PublishBufferAwareTypeDuringShutdown)
{
  using Msg = test_msgs::msg::UnboundedSequences;
  for (int i = 0; i < 5; ++i) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("publish_during_shutdown_" + std::to_string(i));
    auto pub = node->create_publisher<Msg>("publish_during_shutdown", 10);
    auto sub = node->create_subscription<Msg>(
      "publish_during_shutdown", 10, [](Msg::ConstSharedPtr) {});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() {executor.spin();});

    std::atomic<bool> stop{false};
    std::string error;
    std::thread publisher([&]() {
        Msg msg;
        msg.uint8_values.resize(1024);
        while (!stop) {
          try {
            pub->publish(msg);
          } catch (const std::exception & e) {
            error = e.what();
            return;
          }
        }
      });
    // Joins the threads if an assertion returns early.
    RCPPUTILS_SCOPE_EXIT(
    {
      stop = true;
      if (publisher.joinable()) {publisher.join();}
      executor.cancel();
      if (spinner.joinable()) {spinner.join();}
    });

    std::this_thread::sleep_for(500ms);
    rclcpp::shutdown();
    stop = true;
    publisher.join();
    spinner.join();
    EXPECT_TRUE(error.empty()) << "iteration " << i << ": publish threw: " << error;
  }
}
