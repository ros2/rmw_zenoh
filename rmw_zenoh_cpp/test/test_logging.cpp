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

#include <string>

#include "rcutils/env.h"
#include "rcutils/logging.h"

#include "detail/logging.hpp"

using rmw_zenoh_cpp::Logger;

class TestLogging : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(
      rcutils_set_env(
        "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{function_name} {file_name}:{line_number}] {message}"));
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_initialize());
  }

  void TearDown() override
  {
    EXPECT_EQ(RCUTILS_RET_OK, rcutils_logging_shutdown());
  }
};

TEST_F(TestLogging, each_message_reports_its_own_location)
{
  testing::internal::CaptureStderr();
  Logger::get().log_named(
    RCUTILS_LOG_SEVERITY_WARN, "first_function", "first_file.cpp", 10, "rmw_zenoh_cpp",
    "message %d", 1);
  Logger::get().log_named(
    RCUTILS_LOG_SEVERITY_WARN, "second_function", "second_file.cpp", 20, "rmw_zenoh_cpp",
    "message %s", "two");
  const std::string output = testing::internal::GetCapturedStderr();

  EXPECT_NE(std::string::npos, output.find("[first_function first_file.cpp:10] message 1"))
    << output;
  EXPECT_NE(std::string::npos, output.find("[second_function second_file.cpp:20] message two"))
    << output;
}
