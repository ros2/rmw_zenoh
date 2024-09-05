// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "logging.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
Logger & Logger::get()
{
  static Logger logger(RCUTILS_LOG_SEVERITY_INFO);
  return logger;
}

///=============================================================================
Logger::Logger(RCUTILS_LOG_SEVERITY threshold_level)
: threshold_level_(threshold_level)
{}

///=============================================================================
void Logger::set_log_level(RCUTILS_LOG_SEVERITY new_level)
{
  threshold_level_ = new_level;
}

void Logger::log_named(
  RCUTILS_LOG_SEVERITY level,
  const char * function_name,
  const char * file_name,
  size_t line_number,
  const char * name,
  const char * message,
  ...) const
{
  if (level >= this->threshold_level_) {
    rcutils_time_point_value_t now;
    rcutils_ret_t ret = rcutils_system_time_now(&now);
    if (ret != RCUTILS_RET_OK) {
      RCUTILS_SAFE_FWRITE_TO_STDERR("Failed to get timestamp while doing a console logging.\n");
      return;
    }
    static rcutils_log_location_t log_location = {function_name, file_name, line_number};
    va_list args;
    va_start(args, message);
    rcutils_logging_console_output_handler(
      &log_location,
      level,
      name,
      now,
      message,
      &args
    );
  }
}
}  // namespace rmw_zenoh_cpp
