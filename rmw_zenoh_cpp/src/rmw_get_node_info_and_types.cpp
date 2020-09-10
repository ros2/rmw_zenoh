// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/rmw.h"
#include "rmw/names_and_types.h"
#include "rmw/topic_endpoint_info_array.h"

#ifdef __cplusplus
extern "C"
{
#endif

rmw_ret_t
rmw_get_publisher_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)no_demangle;
  (void)topic_names_and_types;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_get_publisher_names_and_types_by_node");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_get_subscriber_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topics_names_and_types)
{
  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)no_demangle;
  (void)topics_names_and_types;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_get_subscriber_names_and_types_by_node");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_get_service_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)service_names_and_types;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_get_service_names_and_types_by_node");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_get_client_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * client_names_and_types)
{
  (void)node;
  (void)allocator;
  (void)node_name;
  (void)node_namespace;
  (void)client_names_and_types;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_get_client_names_and_types_by_node");
  return RMW_RET_ERROR;
}

#ifdef __cplusplus
}  // extern "C"
#endif
