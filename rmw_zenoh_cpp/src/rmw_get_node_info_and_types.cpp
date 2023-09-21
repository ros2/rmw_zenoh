// Copyright 2023 Open Source Robotics Foundation, Inc.
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


#include "rmw/get_node_info_and_types.h"

extern "C"
{
///==============================================================================
// Return all topic names and types for which a given remote node has subscriptions.
rmw_ret_t
rmw_get_subscriber_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  static_cast<void>(node);
  static_cast<void>(allocator);
  static_cast<void>(node_name);
  static_cast<void>(node_namespace);
  static_cast<void>(no_demangle);
  static_cast<void>(topic_names_and_types);
  return RMW_RET_UNSUPPORTED;
}

///==============================================================================
/// Return all topic names and types for which a given remote node has publishers.
rmw_ret_t
rmw_get_publisher_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  static_cast<void>(node);
  static_cast<void>(allocator);
  static_cast<void>(node_name);
  static_cast<void>(node_namespace);
  static_cast<void>(no_demangle);
  static_cast<void>(topic_names_and_types);
  return RMW_RET_UNSUPPORTED;
}

///==============================================================================
/// Return all service names and types for which a given remote node has servers.
rmw_ret_t
rmw_get_service_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  static_cast<void>(node);
  static_cast<void>(allocator);
  static_cast<void>(node_name);
  static_cast<void>(node_namespace);
  static_cast<void>(service_names_and_types);
  return RMW_RET_UNSUPPORTED;
}

///==============================================================================
/// Return all service names and types for which a given remote node has clients.
rmw_ret_t
rmw_get_client_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  static_cast<void>(node);
  static_cast<void>(allocator);
  static_cast<void>(node_name);
  static_cast<void>(node_namespace);
  static_cast<void>(service_names_and_types);
  return RMW_RET_UNSUPPORTED;
}
}  // extern "C"
