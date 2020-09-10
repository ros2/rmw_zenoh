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
#include "rmw/rmw.h"
#include "rmw/names_and_types.h"
#include "rmw/topic_endpoint_info_array.h"

extern "C"
{
rmw_ret_t
rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces)
{
  (void)node;
  (void)node_names;
  (void)node_namespaces;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_get_node_names");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_get_node_names_with_enclaves(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves)
{
  (void)node;
  (void)node_names;
  (void)node_namespaces;
  (void)enclaves;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_get_node_names_with_enclaves");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_count_publishers(const rmw_node_t * node, const char * topic_name, size_t * count)
{
  (void)node;
  (void)topic_name;
  (void)count;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_count_publishers");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_count_subscribers(const rmw_node_t * node, const char * topic_name, size_t * count)
{
  (void)node;
  (void)topic_name;
  (void)count;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_count_subscribers");
  return RMW_RET_ERROR;
}
}  // extern "C"
