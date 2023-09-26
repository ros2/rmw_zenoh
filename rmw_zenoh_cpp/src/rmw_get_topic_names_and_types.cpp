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

#include "rcutils/strdup.h"

#include "rmw/error_handling.h"
#include "rmw/get_topic_names_and_types.h"

#include "rcpputils/scope_exit.hpp"

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
  static_cast<void>(node);
  static_cast<void>(no_demangle);

  rmw_ret_t ret = rmw_names_and_types_init(topic_names_and_types, 1, allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  auto cleanup_names_and_types = rcpputils::make_scope_exit(
    [topic_names_and_types] {
      rmw_ret_t fail_ret = rmw_names_and_types_fini(topic_names_and_types);
      if (fail_ret != RMW_RET_OK) {
        RMW_SAFE_FWRITE_TO_STDERR("failed to cleanup names and types during error handling");
      }
    });

  // topic_names_and_types->names is an rcutils_string_array_t,
  // while topic_names_and_types->types is an rcutils_string_array_t *

  topic_names_and_types->names.data[0] = rcutils_strdup("/chatter", *allocator);
  if (topic_names_and_types->names.data[0] == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate memory for topic names");
    return RMW_RET_BAD_ALLOC;
  }
  auto free_names = rcpputils::make_scope_exit(
    [topic_names_and_types, allocator] {
      allocator->deallocate(topic_names_and_types->names.data[0], allocator->state);
    });

  rcutils_ret_t rcutils_ret = rcutils_string_array_init(
    topic_names_and_types->types, 1,
    allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to allocate memory for topic types");
    return RMW_RET_BAD_ALLOC;
  }
  auto fini_string_array = rcpputils::make_scope_exit(
    [topic_names_and_types] {
      rmw_ret_t fail_ret = rcutils_string_array_fini(topic_names_and_types->types);
      if (fail_ret != RMW_RET_OK) {
        RMW_SAFE_FWRITE_TO_STDERR("failed to cleanup topic types during error handling");
      }
    });

  topic_names_and_types->types[0].data[0] = rcutils_strdup("std_msgs/msg/String", *allocator);
  if (topic_names_and_types->types[0].data[0] == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate memory for topic data");
    return RMW_RET_BAD_ALLOC;
  }
  auto free_types = rcpputils::make_scope_exit(
    [topic_names_and_types, allocator] {
      allocator->deallocate(topic_names_and_types->types[0].data[0], allocator->state);
    });

  free_types.cancel();
  fini_string_array.cancel();
  free_names.cancel();
  cleanup_names_and_types.cancel();

  return RMW_RET_OK;
}
}  // extern "C"
