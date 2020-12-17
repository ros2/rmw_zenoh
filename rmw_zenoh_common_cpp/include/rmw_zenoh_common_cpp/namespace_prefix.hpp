// Copyright 2020 ADLINK, Inc.
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

#ifndef RMW_ZENOH_COMMON_CPP__NAMESPACE_PREFIX_HPP_
#define RMW_ZENOH_COMMON_CPP__NAMESPACE_PREFIX_HPP_

#include <vector>
#include <string>

#include "rmw_zenoh_common_cpp/visibility_control.h"

extern "C"
{
RMW_ZENOH_COMMON_CPP_PUBLIC extern const char * const ros_topic_prefix;
RMW_ZENOH_COMMON_CPP_PUBLIC extern const char * const ros_service_requester_prefix;
RMW_ZENOH_COMMON_CPP_PUBLIC extern const char * const ros_service_response_prefix;

RMW_ZENOH_COMMON_CPP_PUBLIC extern const std::vector<std::string> _ros_prefixes;
}  // extern "C"

/// Returns `name` stripped of `prefix` if exists, if not return "".
/**
 * \param[in] name string that will be stripped from prefix
 * \param[in] prefix prefix to be stripped
 * \return name stripped of prefix, or
 * \return "" if name doesn't start with prefix
 */
RMW_ZENOH_COMMON_CPP_PUBLIC
std::string
_resolve_prefix(const std::string & name, const std::string & prefix);

/// Return the ROS specific prefix if it exists, otherwise "".
RMW_ZENOH_COMMON_CPP_PUBLIC
std::string
_get_ros_prefix_if_exists(const std::string & topic_name);

/// Returns the topic name stripped of and ROS specific prefix if exists.
RMW_ZENOH_COMMON_CPP_PUBLIC
std::string
_strip_ros_prefix_if_exists(const std::string & topic_name);

/// Returns the list of ros prefixes
RMW_ZENOH_COMMON_CPP_PUBLIC
const std::vector<std::string> &
_get_all_ros_prefixes();
#endif  // RMW_ZENOH_COMMON_CPP__NAMESPACE_PREFIX_HPP_
