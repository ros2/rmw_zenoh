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
#include "rcutils/strdup.h"

#include "rmw/rmw.h"
#include "rmw/error_handling.h"

#include "test_msgs/msg/basic_types.h"

#define RMW_IMPLEMENTATION rmw_zenoh_cpp
#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestPublisher, RMW_IMPLEMENTATION) : public ::testing::Test
{
protected:
  void SetUp() override
  {
    init_options = rmw_get_zero_initialized_init_options();
    rmw_ret_t ret = rmw_init_options_init(&init_options, rcutils_get_default_allocator());
    ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
    init_options.enclave = rcutils_strdup("/", rcutils_get_default_allocator());
    ASSERT_STREQ("/", init_options.enclave);
    context = rmw_get_zero_initialized_context();
    ret = rmw_init(&init_options, &context);
    ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
    const char * const node_name = "my_test_node";
    const char * const node_namespace = "/my_test_ns";
    node = rmw_create_node(&context, node_name, node_namespace, 0, false);
    ASSERT_NE(nullptr, node) << rcutils_get_error_string().str;

    options = rmw_get_default_publisher_options();
    ts = ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, BasicTypes);
  }

  void TearDown() override
  {
    rmw_ret_t ret = rmw_destroy_node(node);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ret = rmw_shutdown(&context);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ret = rmw_context_fini(&context);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ret = rmw_init_options_fini(&init_options);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
  }

  rmw_init_options_t init_options;
  rmw_context_t context;
  rmw_node_t * node;

  rmw_publisher_options_t options;
  const rosidl_message_type_support_t * ts;

};

TEST_F(CLASSNAME(TestPublisher, RMW_IMPLEMENTATION), create_and_destroy) {
  const char * const topic_name = "/test";
  rmw_publisher_t * pub =
    rmw_create_publisher(node, ts, topic_name, &rmw_qos_profile_default, &options);
  ASSERT_NE(nullptr, pub) << rmw_get_error_string().str;
  rmw_ret_t ret = rmw_destroy_publisher(node, pub);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;

  const char * const native_topic_name = "test";
  rmw_qos_profile_t native_qos_profile = rmw_qos_profile_default;
  native_qos_profile.avoid_ros_namespace_conventions = true;
  pub = rmw_create_publisher(node, ts, native_topic_name, &native_qos_profile, &options);
  ASSERT_NE(nullptr, pub) << rmw_get_error_string().str;
  ret = rmw_destroy_publisher(node, pub);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
}

TEST_F(CLASSNAME(TestPublisher, RMW_IMPLEMENTATION), create_with_bad_arguments) {
  const char * const topic_name = "/test";
  rmw_publisher_t * pub =
    rmw_create_publisher(nullptr, ts, topic_name, &rmw_qos_profile_default, &options);
  EXPECT_EQ(nullptr, pub);
  rmw_reset_error();

  pub = rmw_create_publisher(node, nullptr, topic_name, &rmw_qos_profile_default, &options);
  EXPECT_EQ(nullptr, pub);
  rmw_reset_error();

  const char * implementation_identifier = node->implementation_identifier;
  node->implementation_identifier = "not-an-rmw-implementation-identifier";
  pub = rmw_create_publisher(node, ts, topic_name, &rmw_qos_profile_default, &options);
  node->implementation_identifier = implementation_identifier;
  EXPECT_EQ(nullptr, pub);
  rmw_reset_error();

  pub = rmw_create_publisher(node, ts, nullptr, &rmw_qos_profile_default, &options);
  ASSERT_EQ(nullptr, pub);
  rmw_reset_error();

  const char * const topic_name_with_spaces = "foo bar";
  pub = rmw_create_publisher(node, ts, topic_name_with_spaces, &rmw_qos_profile_default, &options);
  ASSERT_EQ(nullptr, pub);
  rmw_reset_error();

  const char * const relative_topic_name = "foo";
  pub = rmw_create_publisher(node, ts, relative_topic_name, &rmw_qos_profile_default, &options);
  ASSERT_EQ(nullptr, pub);
  rmw_reset_error();

  pub = rmw_create_publisher(node, ts, topic_name, nullptr, &options);
  ASSERT_EQ(nullptr, pub);
  rmw_reset_error();

  pub = rmw_create_publisher(node, ts, topic_name, &rmw_qos_profile_unknown, &options);
  ASSERT_EQ(nullptr, pub);
  rmw_reset_error();

  pub = rmw_create_publisher(node, ts, topic_name, &rmw_qos_profile_default, nullptr);
  ASSERT_EQ(nullptr, pub);
  rmw_reset_error();

  // Creating and destroying a publisher still succeeds.
  pub = rmw_create_publisher(node, ts, topic_name, &rmw_qos_profile_default, &options);
  ASSERT_NE(nullptr, pub) << rmw_get_error_string().str;
  rmw_ret_t ret = rmw_destroy_publisher(node, pub);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
}

TEST_F(CLASSNAME(TestPublisher, RMW_IMPLEMENTATION), destroy_with_bad_arguments) {
  rmw_publisher_options_t options = rmw_get_default_publisher_options();
  const char * const topic_name = "/test";
  const rosidl_message_type_support_t * ts =
    ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, BasicTypes);
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_default;
  rmw_publisher_t * pub = rmw_create_publisher(node, ts, topic_name, qos_profile, &options);
  ASSERT_NE(nullptr, pub) << rmw_get_error_string().str;

  // Destroying publisher with invalid arguments fails.
  rmw_ret_t ret = rmw_destroy_publisher(nullptr, pub);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rmw_reset_error();

  ret = rmw_destroy_publisher(node, nullptr);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rmw_reset_error();

  const char * implementation_identifier = node->implementation_identifier;
  node->implementation_identifier = "not-an-rmw-implementation-identifier";
  ret = rmw_destroy_publisher(node, pub);
  node->implementation_identifier = implementation_identifier;
  EXPECT_EQ(RMW_RET_INCORRECT_RMW_IMPLEMENTATION, ret);
  rmw_reset_error();

  // Destroying publisher still succeeds.
  ret = rmw_destroy_publisher(node, pub);
  EXPECT_EQ(RMW_RET_OK, ret);
  rmw_reset_error();
}
