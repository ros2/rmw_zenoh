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

#include "detail/rmw_data_types.hpp"

#include "rmw/error_handling.h"
#include "rmw/get_topic_names_and_types.h"

extern "C"
{
///==============================================================================
/// Return all topic names and types in the ROS graph.
// TODO(yadunund): Fix implementation once discovery information can be cached.
rmw_ret_t
rmw_get_topic_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_names_and_types, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache.get_topic_names_and_types(
    allocator, no_demangle, topic_names_and_types);
}
}  // extern "C"
