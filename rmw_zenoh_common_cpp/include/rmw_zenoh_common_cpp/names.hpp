// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef RMW_ZENOH_COMMON_CPP__NAMES_HPP_
#define RMW_ZENOH_COMMON_CPP__NAMES_HPP_

#include <sstream>
#include <string>

// #include "fastrtps/utils/fixed_size_string.hpp"
#include "rmw/types.h"
#include "namespace_prefix.hpp"

/// Construct a topic name.
/**
  * \param[in] prefix Required prefix for topic name.
  * \param[in] base Required name of the topic.
  * \param[in] suffix Optional suffix for topic name.
  */
inline
std::string
_mangle_topic_name(
  const char * prefix,
  const char * base,
  const char * suffix = nullptr)
{
  std::ostringstream topicName;
  if (prefix) {
    topicName << prefix;
  }
  topicName << base;
  if (suffix) {
    topicName << suffix;
  }
  return topicName.str();
}

/// Construct a topic name according to proper conventions.
/**
  * \param[in] qos_profile The QoS profile for the topic.
  * \param[in] prefix Required prefix for topic name.
  * \param[in] base Required name of the topic.
  * \param[in] suffix Optional suffix for topic name.
  */
inline
std::string
_create_topic_name(
  const rmw_qos_profile_t * qos_profile,
  const char * prefix,
  const char * base,
  const char * suffix = nullptr)
{
  assert(qos_profile);
  assert(base);
  if (qos_profile->avoid_ros_namespace_conventions) {
    prefix = nullptr;
  }
  return _mangle_topic_name(prefix, base, suffix);
}

#endif  // RMW_ZENOH_COMMON_CPP__NAMES_HPP_
