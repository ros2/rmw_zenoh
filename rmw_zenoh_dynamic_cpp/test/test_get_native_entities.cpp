// Copyright 2020 ADLINK, Inc.
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

#include "osrf_testing_tools_cpp/scope_exit.hpp"

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

#include "rmw_zenoh_dynamic_cpp/get_client.hpp"
#include "rmw_zenoh_dynamic_cpp/get_session.hpp"
#include "rmw_zenoh_dynamic_cpp/get_publisher.hpp"
#include "rmw_zenoh_dynamic_cpp/get_service.hpp"
#include "rmw_zenoh_dynamic_cpp/get_subscriber.hpp"

#include "test_msgs/msg/basic_types.h"
#include "test_msgs/srv/basic_types.h"

class TestNativeEntities : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rmw_init_options_t options = rmw_get_zero_initialized_init_options();
    rmw_ret_t ret = rmw_init_options_init(&options, rcutils_get_default_allocator());
    ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    {
      rmw_ret_t ret = rmw_init_options_fini(&options);
      EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    });
    options.enclave = rcutils_strdup("/", rcutils_get_default_allocator());
    ASSERT_STREQ("/", options.enclave);
    ret = rmw_init(&options, &context);
    ASSERT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    constexpr char node_name[] = "my_node";
    constexpr char node_namespace[] = "/my_ns";
    node = rmw_create_node(&context, node_name, node_namespace, 1, false);
    ASSERT_NE(nullptr, node) << rmw_get_error_string().str;
  }

  void TearDown() override
  {
    rmw_ret_t ret = rmw_destroy_node(node);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ret = rmw_shutdown(&context);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
    ret = rmw_context_fini(&context);
    EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
  }

  rmw_context_t context{rmw_get_zero_initialized_context()};
  rmw_node_t * node{nullptr};
};

TEST_F(TestNativeEntities, get_sessions) {
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_session(nullptr));

  const char * implementation_identifier = node->implementation_identifier;
  node->implementation_identifier = "not-an-rmw-implementation-identifier";
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_session(node));
  node->implementation_identifier = implementation_identifier;

  EXPECT_NE(nullptr, rmw_zenoh_dynamic_cpp::get_session(node));
}

TEST_F(TestNativeEntities, get_publisher) {
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_publisher(nullptr));

  const rosidl_message_type_support_t * ts =
    ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, BasicTypes);
  constexpr char topic_name[] = "/test";
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  rmw_publisher_options_t options = rmw_get_default_publisher_options();
  rmw_publisher_t * pub = rmw_create_publisher(node, ts, topic_name, &qos_profile, &options);
  ASSERT_NE(nullptr, pub) << rmw_get_error_string().str;

  const char * implementation_identifier = pub->implementation_identifier;
  pub->implementation_identifier = "not-an-rmw-implementation-identifier";
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_publisher(pub));
  pub->implementation_identifier = implementation_identifier;

  EXPECT_NE(nullptr, rmw_zenoh_dynamic_cpp::get_publisher(pub));

  rmw_ret_t ret = rmw_destroy_publisher(node, pub);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
}

TEST_F(TestNativeEntities, get_subscriber) {
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_subscriber(nullptr));

  const rosidl_message_type_support_t * ts =
    ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, BasicTypes);
  constexpr char topic_name[] = "/test";
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  rmw_subscription_options_t options = rmw_get_default_subscription_options();
  rmw_subscription_t * sub =
    rmw_create_subscription(node, ts, topic_name, &qos_profile, &options);
  ASSERT_NE(nullptr, sub) << rmw_get_error_string().str;

  const char * implementation_identifier = sub->implementation_identifier;
  sub->implementation_identifier = "not-an-rmw-implementation-identifier";
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_subscriber(sub));
  sub->implementation_identifier = implementation_identifier;

  EXPECT_NE(nullptr, rmw_zenoh_dynamic_cpp::get_subscriber(sub));

  rmw_ret_t ret = rmw_destroy_subscription(node, sub);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
}

TEST_F(TestNativeEntities, get_service) {
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_request_subscriber(nullptr));
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_response_publisher(nullptr));

  const rosidl_service_type_support_t * ts =
    ROSIDL_GET_SRV_TYPE_SUPPORT(test_msgs, srv, BasicTypes);
  constexpr char service_name[] = "/test";
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  rmw_service_t * srv = rmw_create_service(node, ts, service_name, &qos_profile);
  ASSERT_NE(nullptr, srv) << rmw_get_error_string().str;

  const char * implementation_identifier = srv->implementation_identifier;
  srv->implementation_identifier = "not-an-rmw-implementation-identifier";
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_request_subscriber(srv));
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_response_publisher(srv));
  srv->implementation_identifier = implementation_identifier;

  EXPECT_NE(nullptr, rmw_zenoh_dynamic_cpp::get_request_subscriber(srv));
  EXPECT_NE(nullptr, rmw_zenoh_dynamic_cpp::get_response_publisher(srv));

  rmw_ret_t ret = rmw_destroy_service(node, srv);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
}

TEST_F(TestNativeEntities, get_client) {
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_request_publisher(nullptr));
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_response_subscriber(nullptr));

  const rosidl_service_type_support_t * ts =
    ROSIDL_GET_SRV_TYPE_SUPPORT(test_msgs, srv, BasicTypes);
  constexpr char service_name[] = "/test";
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  rmw_client_t * client = rmw_create_client(node, ts, service_name, &qos_profile);
  ASSERT_NE(nullptr, client) << rmw_get_error_string().str;

  const char * implementation_identifier = client->implementation_identifier;
  client->implementation_identifier = "not-an-rmw-implementation-identifier";
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_request_publisher(client));
  EXPECT_EQ(nullptr, rmw_zenoh_dynamic_cpp::get_response_subscriber(client));
  client->implementation_identifier = implementation_identifier;

  EXPECT_NE(nullptr, rmw_zenoh_dynamic_cpp::get_request_publisher(client));
  EXPECT_NE(nullptr, rmw_zenoh_dynamic_cpp::get_response_subscriber(client));

  rmw_ret_t ret = rmw_destroy_client(node, client);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
}
