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

#include <fmt/chrono.h>
#include <fmt/color.h>
#include <fmt/printf.h>

#include <iostream>
#include <mutex>
#include <string>
#include <sstream>

namespace rmw_zenoh_cpp
{
///=============================================================================
enum LogLevel
{
  RMW_ZENOH_LOG_LEVEL_DEBUG,
  RMW_ZENOH_LOG_LEVEL_INFO,
  RMW_ZENOH_LOG_LEVEL_WARN,
  RMW_ZENOH_LOG_LEVEL_ERROR,
};

///=============================================================================
class Logger
{
public:
  // Get a static reference to the logger.
  static Logger & get();

  // Set the threshold log level.
  void set_log_level(LogLevel new_level);

  // Log to the console.
  template<typename ... Args>
  void log_named(
    LogLevel level,
    const std::string & name,
    const std::string & message,
    const Args &... args) const
  {
    if (level >= threshold_level_) {
      const auto time_now = std::chrono::system_clock::now().time_since_epoch();
      const auto seconds =
        std::chrono::duration_cast<std::chrono::seconds>(time_now).count();
      auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        time_now - std::chrono::seconds(seconds)).count();
      fmt::print(
        fmt::fg(level_to_color(level)),
        "[{}] [{}.{}] [{}]: {}\n",
        level_to_string(level),
        seconds,
        nanoseconds,
        name,
        fmt::sprintf(message, args ...)
      );
    }
  }

private:
  LogLevel threshold_level_;
  explicit Logger(LogLevel threshold_level);
  fmt::color level_to_color(LogLevel level) const;
  std::string level_to_string(LogLevel level) const;
};

}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__LOGGING_HPP_
