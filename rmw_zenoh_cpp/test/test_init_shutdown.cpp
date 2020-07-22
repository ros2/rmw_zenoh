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
#include "rcutils/error_handling.h"
#include "rcutils/strdup.h"

#include "rmw/rmw.h"

#define RMW_IMPLEMENTATION rmw_zenoh_cpp
#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestInitShutdown, RMW_IMPLEMENTATION) : public ::testing::Test
{
protected:
  void SetUp() override
  {
    options = rmw_get_zero_initialized_init_options();
    rmw_ret_t ret = rmw_init_options_init(&options, rcutils_get_default_allocator());
    ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
    options.enclave = rcutils_strdup("/", rcutils_get_default_allocator());
    ASSERT_STREQ("/", options.enclave);
  }

  void TearDown() override
  {
    rmw_ret_t ret = rmw_init_options_fini(&options);
    EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  }

  rmw_init_options_t options;
};

TEST_F(CLASSNAME(TestInitShutdown, RMW_IMPLEMENTATION), init_with_bad_arguments) {
  rmw_context_t context = rmw_get_zero_initialized_context();
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_init(nullptr, &context));
  rcutils_reset_error();

  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_init(&options, nullptr));
  rcutils_reset_error();

  rmw_init_options_t invalid_options = rmw_get_zero_initialized_init_options();
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_init(&invalid_options, &context));
  rcutils_reset_error();

  const char * implementation_identifier = options.implementation_identifier;
  options.implementation_identifier = "not-a-real-rmw-implementation-identifier";
  EXPECT_EQ(RMW_RET_INCORRECT_RMW_IMPLEMENTATION, rmw_init(&options, &context));
  options.implementation_identifier = implementation_identifier;
  rcutils_reset_error();

  char * enclave = options.enclave;
  options.enclave = nullptr;
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_init(&options, &context));
  options.enclave = enclave;
  rcutils_reset_error();

  // Initialization, shutdown, and finalization should still succeed
  rmw_ret_t ret = rmw_init(&options, &context);
  ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  ret = rmw_shutdown(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  ret = rmw_context_fini(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
}

TEST_F(CLASSNAME(TestInitShutdown, RMW_IMPLEMENTATION), shutdown_with_bad_arguments) {
  rmw_ret_t ret = rmw_shutdown(nullptr);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  rmw_context_t context = rmw_get_zero_initialized_context();
  ret = rmw_shutdown(&context);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  ret = rmw_init(&options, &context);
  ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  const char * implementation_identifier = context.implementation_identifier;
  context.implementation_identifier = "not-a-real-rmw-implementation-identifier";
  EXPECT_EQ(RMW_RET_INCORRECT_RMW_IMPLEMENTATION, rmw_shutdown(&context));
  context.implementation_identifier = implementation_identifier;
  rcutils_reset_error();

  // Shutdown, and finalization should still succeed
  ret = rmw_shutdown(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  ret = rmw_context_fini(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
}

TEST_F(CLASSNAME(TestInitShutdown, RMW_IMPLEMENTATION), context_fini_with_bad_arguments) {
  rmw_ret_t ret = rmw_context_fini(nullptr);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  rmw_context_t context = rmw_get_zero_initialized_context();
  ret = rmw_context_fini(&context);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  ret = rmw_init(&options, &context);
  ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  const char * implementation_identifier = context.implementation_identifier;
  context.implementation_identifier = "not-a-real-rmw-implementation-identifier";
  EXPECT_EQ(RMW_RET_INCORRECT_RMW_IMPLEMENTATION, rmw_context_fini(&context));
  context.implementation_identifier = implementation_identifier;
  rcutils_reset_error();

  // Shutdown and finalization should still succeed
  ret = rmw_shutdown(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  ret = rmw_context_fini(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
}

TEST_F(CLASSNAME(TestInitShutdown, RMW_IMPLEMENTATION), init_shutdown) {
  rmw_context_t context = rmw_get_zero_initialized_context();
  rmw_ret_t ret = rmw_init(&options, &context);
  ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  // Initialization twice should fail
  ret = rmw_init(&options, &context);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  // Finalization w/o shutdown should fail
  ret = rmw_context_fini(&context);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);
  rcutils_reset_error();

  ret = rmw_shutdown(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  // Shutdown twice should succeed
  ret = rmw_shutdown(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  ret = rmw_context_fini(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;

  // Finalization twice should fail
  ret = rmw_context_fini(&context);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, ret);

  // Initialization, shutdown, and finalization should still succeed
  ret = rmw_init(&options, &context);
  ASSERT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  ret = rmw_shutdown(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
  ret = rmw_context_fini(&context);
  EXPECT_EQ(RMW_RET_OK, ret) << rcutils_get_error_string().str;
}
