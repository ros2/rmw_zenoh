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

rmw_ret_t
rmw_get_serialized_message_size(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  size_t * size)
{
  (void)type_support;
  (void)message_bounds;
  (void)size;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_common_cpp", "rmw_get_serialized_message_size");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  (void)ros_message;
  (void)type_support;
  (void)serialized_message;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_common_cpp", "rmw_serialize");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_support,
  void * ros_message)
{
  (void)serialized_message;
  (void)type_support;
  (void)ros_message;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_common_cpp", "rmw_deserialize");
  return RMW_RET_ERROR;
}
