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

#include "osrf_testing_tools_cpp/scope_exit.hpp"

#include "rcutils/allocator.h"
#include "rcutils/error_handling.h"

#include "rmw/rmw.h"

#include "./allocator_testing_utils.h"

#define RMW_IMPLEMENTATION rmw_zenoh_cpp
#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestInitOptions, RMW_IMPLEMENTATION) : public ::testing::Test {};

TEST_F(CLASSNAME(TestInitOptions, RMW_IMPLEMENTATION), init_copy_fini) {
  rmw_init_options_t src_options = rmw_get_zero_initialized_init_options();
  rmw_ret_t ret = rmw_init_options_init(&src_options, rcutils_get_default_allocator());
  ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  // Initializing twice fails.
  ret = rmw_init_options_init(&src_options, rcutils_get_default_allocator());
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  rmw_init_options_t dst_options = rmw_get_zero_initialized_init_options();
  ret = rmw_init_options_copy(&src_options, &dst_options);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  // Copying twice fails.
  ret = rmw_init_options_copy(&src_options, &dst_options);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret) << rcutils_get_error_string().str;
  rcutils_reset_error();

  ret = rmw_init_options_fini(&dst_options);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  ret = rmw_init_options_fini(&src_options);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  // Finalizing twice fails.
  ret = rmw_init_options_fini(&src_options);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();
}

TEST_F(CLASSNAME(TestInitOptions, RMW_IMPLEMENTATION), init_with_bad_arguments) {
  rmw_init_options_t options = rmw_get_zero_initialized_init_options();
  rmw_ret_t ret = rmw_init_options_init(&options, rcutils_get_zero_initialized_allocator());
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  ret = rmw_init_options_init(nullptr, rcutils_get_default_allocator());
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  // Initialization and finalization should still succeed.
  ret = rmw_init_options_init(&options, rcutils_get_default_allocator());
  ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  ret = rmw_init_options_fini(&options);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
}

TEST_F(CLASSNAME(TestInitOptions, RMW_IMPLEMENTATION), copy_with_bad_arguments) {
  rmw_init_options_t src_options = rmw_get_zero_initialized_init_options();
  rmw_init_options_t dst_options = rmw_get_zero_initialized_init_options();

  rmw_ret_t ret = rmw_init_options_copy(nullptr, &dst_options);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  ret = rmw_init_options_copy(&src_options, nullptr);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  ret = rmw_init_options_copy(&src_options, &dst_options);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  ret = rmw_init_options_init(&src_options, rcutils_get_default_allocator());
  ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rmw_ret_t ret = rmw_init_options_fini(&src_options);
    EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  });

  const char * implementation_identifier = src_options.implementation_identifier;
  src_options.implementation_identifier = "not-an-rmw-implementation-identifier";
  ret = rmw_init_options_copy(&src_options, &dst_options);
  EXPECT_EQ(RMW_RET_INCORRECT_RMW_IMPLEMENTATION, ret);
  src_options.implementation_identifier = implementation_identifier;
  rcutils_reset_error();

  // Initialization and finalization should still succeed.
  ret = rmw_init_options_init(&dst_options, rcutils_get_default_allocator());
  ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  ret = rmw_init_options_fini(&dst_options);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
}

TEST_F(CLASSNAME(TestInitOptions, RMW_IMPLEMENTATION), fini_with_bad_arguments) {
  rmw_init_options_t options = rmw_get_zero_initialized_init_options();
  rmw_ret_t ret = rmw_init_options_fini(&options);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  ret = rmw_init_options_fini(nullptr);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  ret = rmw_init_options_init(&options, rcutils_get_default_allocator());
  ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  const char * implementation_identifier = options.implementation_identifier;
  options.implementation_identifier = "not-an-rmw-implementation-identifier";
  ret = rmw_init_options_fini(&options);
  EXPECT_EQ(RMW_RET_INCORRECT_RMW_IMPLEMENTATION, ret);
  options.implementation_identifier = implementation_identifier;
  rcutils_reset_error();

  ret = rmw_init_options_fini(&options);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  // Initialization and finalization should still succeed.
  ret = rmw_init_options_init(&options, rcutils_get_default_allocator());
  ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  ret = rmw_init_options_fini(&options);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
}
