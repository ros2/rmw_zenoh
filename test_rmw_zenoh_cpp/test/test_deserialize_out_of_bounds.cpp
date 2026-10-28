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

#include <cstdint>
#include <cstring>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <test_msgs/msg/bounded_sequences.hpp>

// Regression test: data that declares a bounded sequence longer than its
// bound must be rejected with an error.
//
// The generated code throws std::length_error when it resizes the sequence.
// Service requests, client responses and rmw_deserialize() all go through the
// same TypeSupport::deserialize_ros_message(), which only caught Fast CDR
// exceptions, so the std::length_error escaped through the rmw API and a
// malformed service request or response aborted the process.
TEST(TestDeserializeOutOfBounds, BoundedSequenceLongerThanItsBound)
{
  rclcpp::Serialization<test_msgs::msg::BoundedSequences> serialization;
  test_msgs::msg::BoundedSequences message;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);

  // Every sequence is empty, so the length of the first member, bool_values
  // (bool[<=3]), is the four bytes after the encapsulation header.
  auto & raw = serialized.get_rcl_serialized_message();
  ASSERT_GE(raw.buffer_length, 8u);
  const uint32_t too_long = 4;
  std::memcpy(raw.buffer + 4, &too_long, sizeof(too_long));

  test_msgs::msg::BoundedSequences result;
  EXPECT_THROW(
    serialization.deserialize_message(&serialized, &result),
    rclcpp::exceptions::RCLError);
}
