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

#ifndef DETAIL__ENDPOINT_INFO_HPP_
#define DETAIL__ENDPOINT_INFO_HPP_

#include <cstring>
#include <string>
#include <utility>

#include "liveliness_utils.hpp"

#include "rmw/types.h"
#include "rmw/topic_endpoint_info.h"
#include "rosidl_runtime_c/type_hash.h"

namespace rmw_zenoh_cpp
{

/// Owning storage for endpoint info, keeping strings alive for the
/// non-owning const char * pointers inside rmw_topic_endpoint_info_t.
struct EndpointInfoStorage
{
  rmw_topic_endpoint_info_t info{};
  std::string node_name;
  std::string node_namespace;
  std::string topic_type;

  EndpointInfoStorage() = default;

  EndpointInfoStorage(const EndpointInfoStorage & other)
  : info(other.info),
    node_name(other.node_name),
    node_namespace(other.node_namespace),
    topic_type(other.topic_type)
  {
    bind_strings();
  }

  EndpointInfoStorage(EndpointInfoStorage && other) noexcept
  : info(other.info),
    node_name(std::move(other.node_name)),
    node_namespace(std::move(other.node_namespace)),
    topic_type(std::move(other.topic_type))
  {
    bind_strings();
  }

  EndpointInfoStorage & operator=(const EndpointInfoStorage & other)
  {
    if (this != &other) {
      info = other.info;
      node_name = other.node_name;
      node_namespace = other.node_namespace;
      topic_type = other.topic_type;
      bind_strings();
    }
    return *this;
  }

  EndpointInfoStorage & operator=(EndpointInfoStorage && other) noexcept
  {
    if (this != &other) {
      info = other.info;
      node_name = std::move(other.node_name);
      node_namespace = std::move(other.node_namespace);
      topic_type = std::move(other.topic_type);
      bind_strings();
    }
    return *this;
  }

  /// Point the strings in info at this object's members.
  void bind_strings()
  {
    info.node_name = node_name.c_str();
    info.node_namespace = node_namespace.c_str();
    info.topic_type = topic_type.c_str();
  }
};

/// Build an EndpointInfoStorage from a liveliness Entity.
/// This is a shared helper that replaces the duplicate lambdas previously
/// defined in rmw_publisher_data.cpp and rmw_subscription_data.cpp.
inline EndpointInfoStorage build_endpoint_info_from_entity(
  const liveliness::Entity & entity,
  rmw_endpoint_type_t endpoint_type)
{
  EndpointInfoStorage storage;
  storage.node_name = entity.node_name();
  storage.node_namespace = entity.node_namespace();
  const auto & topic_info = entity.topic_info();
  if (topic_info.has_value()) {
    storage.topic_type = topic_info->type_;
    storage.info.qos_profile = topic_info->qos_;
    storage.info.topic_type_hash = rosidl_get_zero_initialized_type_hash();
    (void)rosidl_parse_type_hash_string(
      topic_info->type_hash_.c_str(),
      &storage.info.topic_type_hash);
  }
  storage.bind_strings();
  storage.info.endpoint_type = endpoint_type;

  auto gid_array = entity.copy_gid();
  std::memcpy(storage.info.endpoint_gid, gid_array.data(), RMW_GID_STORAGE_SIZE);
  return storage;
}

}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__ENDPOINT_INFO_HPP_
