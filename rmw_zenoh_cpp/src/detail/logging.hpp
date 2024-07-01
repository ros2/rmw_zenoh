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

#ifndef DETAIL__LOGGING_HPP_
#define DETAIL__LOGGING_HPP_

#include <rcutils/logging.h>

#include <iostream>
#include <mutex>
#include <string>
#include <sstream>

namespace rmw_zenoh_cpp
{
///=============================================================================
class Logger
{
public:
  // Get a static reference to the logger.
  static Logger & get();

  // Set the threshold log level.
  void set_log_level(RCUTILS_LOG_SEVERITY new_level);

  // Log to the console.
  void log_named(
    RCUTILS_LOG_SEVERITY level,
    const char * name,
    const char * message,
    ...) const
  {
    if (level >= threshold_level_) {
      rcutils_time_point_value_t now;
      rcutils_ret_t ret = rcutils_system_time_now(&now);
      if (ret != RCUTILS_RET_OK) {
        RCUTILS_SAFE_FWRITE_TO_STDERR("Failed to get timestamp while doing a console logging.\n");
        return;
      }
      static rcutils_log_location_t log_location = {__func__, __FILE__, __LINE__};
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

private:
  RCUTILS_LOG_SEVERITY threshold_level_;
  explicit Logger(RCUTILS_LOG_SEVERITY threshold_level);
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__LOGGING_HPP_
