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

#ifndef DETAIL__LOGGING_MACROS_HPP_
#define DETAIL__LOGGING_MACROS_HPP_

#include "logging.hpp"

///=============================================================================
// We define custom logging marcos to log to console as it was discovered
// that relying on RCUTILS_LOG_X_NAMED functions also end up logging over /rosout
// which can lead to deadlocks in rmw_zenoh especially when multiple threads
// invoke GraphCache::parse_put() and GraphCache::parse_del() functions.
// See https://github.com/ros2/rmw_zenoh/issues/182 for more details.
#define RMW_ZENOH_LOG_DEBUG_NAMED(...) {rmw_zenoh_cpp::Logger::get().log_named( \
      RCUTILS_LOG_SEVERITY_DEBUG, __func__, __FILE__, __LINE__, __VA_ARGS__);}
#define RMW_ZENOH_LOG_ERROR_NAMED(...) {rmw_zenoh_cpp::Logger::get().log_named( \
      RCUTILS_LOG_SEVERITY_ERROR, __func__, __FILE__, __LINE__, __VA_ARGS__);}
#define RMW_ZENOH_LOG_FATAL_NAMED(...) {rmw_zenoh_cpp::Logger::get().log_named( \
      RCUTILS_LOG_SEVERITY_FATAL, __func__, __FILE__, __LINE__, __VA_ARGS__);}
#define RMW_ZENOH_LOG_INFO_NAMED(...) {rmw_zenoh_cpp::Logger::get().log_named( \
      RCUTILS_LOG_SEVERITY_INFO, __func__, __FILE__, __LINE__, __VA_ARGS__);}
#define RMW_ZENOH_LOG_WARN_NAMED(...) {rmw_zenoh_cpp::Logger::get().log_named( \
      RCUTILS_LOG_SEVERITY_WARN, __func__, __FILE__, __LINE__, __VA_ARGS__);}

#endif   // DETAIL__LOGGING_MACROS_HPP_
