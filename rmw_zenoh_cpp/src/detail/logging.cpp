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

#include <sstream>

namespace rmw_zenoh_cpp
{
///=============================================================================
Logger & Logger::get()
{
  static Logger logger(RMW_ZENOH_LOG_LEVEL_INFO);
  return logger;
}

///=============================================================================
Logger::Logger(LogLevel threshold_level)
: threshold_level_(threshold_level)
{}

///=============================================================================
fmt::color Logger::level_to_color(LogLevel level) const
{
  switch (level) {
    case RMW_ZENOH_LOG_LEVEL_DEBUG:
      return fmt::color::light_green;
    case RMW_ZENOH_LOG_LEVEL_ERROR:
      return fmt::color::indian_red;
    case RMW_ZENOH_LOG_LEVEL_INFO:
      return fmt::color::light_blue;
    case RMW_ZENOH_LOG_LEVEL_WARN:
      return fmt::color::light_yellow;
    default:
      return fmt::color::white;
  }
}

///=============================================================================
std::string Logger::level_to_string(LogLevel level) const
{
  switch (level) {
    case RMW_ZENOH_LOG_LEVEL_DEBUG:
      return "DEBUG";
    case RMW_ZENOH_LOG_LEVEL_ERROR:
      return "ERROR";
    case RMW_ZENOH_LOG_LEVEL_INFO:
      return "INFO";
    case RMW_ZENOH_LOG_LEVEL_WARN:
      return "WARN";
    default:
      return "UNKOWN";
  }
}

///=============================================================================
void Logger::set_log_level(LogLevel new_level)
{
  threshold_level_ = new_level;
}
}  // namespace rmw_zenoh_cpp
