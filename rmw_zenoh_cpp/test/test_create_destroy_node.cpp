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

#include "rmw/error_handling.h"
#include "rmw/rmw.h"


#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestNodeConstructionDestruction, RMW_IMPLEMENTATION) : public ::testing::Test
{
protected:
  void SetUp() override
  {
    options = rmw_get_zero_initialized_init_options();
    rmw_ret_t ret = rmw_init_options_init(&options, rcutils_get_default_allocator());
    ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
    options.enclave = rcutils_strdup("/", rcutils_get_default_allocator());
    ASSERT_STREQ("/", options.enclave);
    context = rmw_get_zero_initialized_context();
    ret = rmw_init(&options, &context);
    ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  }

  void TearDown() override
  {
    rmw_ret_t ret = rmw_shutdown(&context);
    EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
    ret = rmw_context_fini(&context);
    EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
    ret = rmw_init_options_fini(&options);
    EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  }

  rmw_init_options_t options;
  rmw_context_t context;
};

TEST_F(CLASSNAME(TestNodeConstructionDestruction, RMW_IMPLEMENTATION), create_with_bad_arguments) {
  const char * const node_name = "my_node";
  const char * const node_namespace = "/my_ns";

  rmw_node_t * node = rmw_create_node(nullptr, node_name, node_namespace, 0, false);
  EXPECT_EQ(nullptr, node);
  rmw_reset_error();

  rmw_context_t invalid_context = rmw_get_zero_initialized_context();
  node = rmw_create_node(&invalid_context, node_name, node_namespace, 0, false);
  EXPECT_EQ(nullptr, node);
  rmw_reset_error();

  const char * const name_with_spaces = "foo bar";

  node = rmw_create_node(&context, nullptr, node_namespace, 0, false);
  EXPECT_EQ(nullptr, node);
  rmw_reset_error();

  node = rmw_create_node(&context, name_with_spaces, node_namespace, 0, false);
  EXPECT_EQ(nullptr, node);
  rmw_reset_error();

  node = rmw_create_node(&context, node_name, nullptr, 0, false);
  EXPECT_EQ(nullptr, node);
  rmw_reset_error();

  node = rmw_create_node(&context, node_name, name_with_spaces, 0, false);
  EXPECT_EQ(nullptr, node);
  rmw_reset_error();

  const char * implementation_identifier = context.implementation_identifier;
  context.implementation_identifier = "not-an-rmw-implementation-identifier";
  node = rmw_create_node(&context, node_name, node_namespace, 0, false);
  EXPECT_EQ(nullptr, node);
  context.implementation_identifier = implementation_identifier;
  rmw_reset_error();

  rmw_ret_t ret = rmw_shutdown(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;

  node = rmw_create_node(&context, node_name, node_namespace, 0, false);
  EXPECT_EQ(nullptr, node);
  rmw_reset_error();
}

TEST_F(CLASSNAME(TestNodeConstructionDestruction, RMW_IMPLEMENTATION), destroy_with_bad_arguments) {
  rmw_ret_t ret = rmw_destroy_node(nullptr);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rmw_reset_error();

  const char * const node_name = "my_node";
  const char * const node_namespace = "/my_ns";
  rmw_node_t * node = rmw_create_node(&context, node_name, node_namespace, 0, false);
  ASSERT_NE(nullptr, node) << rmw_get_error_string().str;

  const char * implementation_identifier = node->implementation_identifier;
  node->implementation_identifier = "not-an-rmw-implementation-identifier";
  ret = rmw_destroy_node(node);
  EXPECT_EQ(RMW_RET_INCORRECT_RMW_IMPLEMENTATION, ret);
  rmw_reset_error();
  node->implementation_identifier = implementation_identifier;

  ret = rmw_destroy_node(node);
  EXPECT_EQ(RMW_RET_OK, ret) << rmw_get_error_string().str;
}

TEST_F(CLASSNAME(TestNodeConstructionDestruction, RMW_IMPLEMENTATION), create_and_destroy) {
  const char * const node_name = "my_node";
  const char * const node_namespace = "/my_ns";
  rmw_node_t * node = rmw_create_node(&context, node_name, node_namespace, 0, false);
  ASSERT_NE(nullptr, node) << rmw_get_error_string().str;
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_node(node)) << rmw_get_error_string().str;
}
