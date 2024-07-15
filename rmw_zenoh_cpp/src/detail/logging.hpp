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
    const char * function_name,
    const char * file_name,
    size_t line_number,
    const char * name,
    const char * message,
    ...) const;

private:
  RCUTILS_LOG_SEVERITY threshold_level_;
  explicit Logger(RCUTILS_LOG_SEVERITY threshold_level);
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__LOGGING_HPP_
