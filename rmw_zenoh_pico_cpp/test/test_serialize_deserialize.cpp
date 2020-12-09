// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rcutils/allocator.h"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "rmw/rmw.h"
#include "rmw/error_handling.h"

#include "test_msgs/msg/basic_types.h"
#include "test_msgs/msg/basic_types.hpp"

#include "./allocator_testing_utils.h"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestSerializeDeserialize, RMW_IMPLEMENTATION) : public ::testing::Test
{
};

TEST_F(CLASSNAME(TestSerializeDeserialize, RMW_IMPLEMENTATION), serialize_with_bad_arguments) {
  const rosidl_message_type_support_t * ts{
    ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, BasicTypes)};
  test_msgs__msg__BasicTypes input_message{};
  ASSERT_TRUE(test_msgs__msg__BasicTypes__init(&input_message));
  rcutils_allocator_t failing_allocator = get_failing_allocator();
  rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(
    RMW_RET_OK, rmw_serialized_message_init(
      &serialized_message, 0lu, &failing_allocator)) << rmw_get_error_string().str;

  EXPECT_NE(RMW_RET_OK, rmw_serialize(&input_message, ts, &serialized_message));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&serialized_message)) <<
    rmw_get_error_string().str;
}

TEST_F(CLASSNAME(TestSerializeDeserialize, RMW_IMPLEMENTATION), clean_round_trip_for_c_message) {
  const rosidl_message_type_support_t * ts{
    ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, BasicTypes)};
  test_msgs__msg__BasicTypes input_message{};
  test_msgs__msg__BasicTypes output_message{};
  ASSERT_TRUE(test_msgs__msg__BasicTypes__init(&input_message));
  ASSERT_TRUE(test_msgs__msg__BasicTypes__init(&output_message));
  rcutils_allocator_t default_allocator = rcutils_get_default_allocator();
  rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(
    RMW_RET_OK, rmw_serialized_message_init(
      &serialized_message, 0lu, &default_allocator)) << rmw_get_error_string().str;

  // Make input_message not equal to output_message.
  input_message.bool_value = !output_message.bool_value;
  input_message.int16_value = output_message.int16_value - 1;
  input_message.uint32_value = output_message.uint32_value + 1000000;

  rmw_ret_t ret = rmw_serialize(&input_message, ts, &serialized_message);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
  EXPECT_NE(nullptr, serialized_message.buffer);
  EXPECT_GT(serialized_message.buffer_length, 0lu);

  ret = rmw_deserialize(&serialized_message, ts, &output_message);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
  EXPECT_EQ(input_message.bool_value, output_message.bool_value);
  EXPECT_EQ(input_message.int16_value, output_message.int16_value);
  EXPECT_EQ(input_message.uint32_value, output_message.uint32_value);

  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&serialized_message)) <<
    rmw_get_error_string().str;
}

TEST_F(CLASSNAME(TestSerializeDeserialize, RMW_IMPLEMENTATION), clean_round_trip_for_cpp_message) {
  const rosidl_message_type_support_t * ts =
    rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::BasicTypes>();
  test_msgs::msg::BasicTypes input_message{};
  test_msgs::msg::BasicTypes output_message{};
  rcutils_allocator_t default_allocator = rcutils_get_default_allocator();
  rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(
    RMW_RET_OK, rmw_serialized_message_init(
      &serialized_message, 0lu, &default_allocator)) << rmw_get_error_string().str;

  // Make input_message not equal to output_message.
  input_message.bool_value = !output_message.bool_value;
  input_message.int16_value = output_message.int16_value - 1;
  input_message.uint32_value = output_message.uint32_value + 1000000;

  rmw_ret_t ret = rmw_serialize(&input_message, ts, &serialized_message);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
  EXPECT_NE(nullptr, serialized_message.buffer);
  EXPECT_GT(serialized_message.buffer_length, 0lu);

  ret = rmw_deserialize(&serialized_message, ts, &output_message);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
  EXPECT_EQ(input_message, output_message);

  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&serialized_message)) <<
    rmw_get_error_string().str;
}
