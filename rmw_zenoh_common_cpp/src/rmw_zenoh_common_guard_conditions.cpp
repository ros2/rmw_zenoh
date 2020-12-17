// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

// This file is significantly modified from:
// https://github.com/ros2/rmw_fastrtps/blob/112c6e14c30dc53e5f9112e03db242c6360b14ff/rmw_fastrtps_shared_cpp/src/rmw_guard_condition.cpp

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/event.h"
#include "rmw/ret_types.h"

#include "rcutils/logging_macros.h"

#include "impl/guard_condition_impl.hpp"

#include "rmw_zenoh_common_cpp/rmw_zenoh_common.h"

/// CREATE GUARD CONDITION =====================================================
// Create a guard condition and return a handle to that guard condition.
rmw_guard_condition_t *
rmw_zenoh_common_create_guard_condition(
  rmw_context_t * context,
  const char * const eclipse_zenoh_identifier)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "rmw_create_guard_condition");

  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr);

  // NOTE(CH3): Unfortunately we can't do custom allocation here because the destruction method
  // does not pass in a context from which we can draw an allocator from
  // TODO(CH3): Once https://github.com/ros2/rmw/issues/260 gets resolved, use an allocator instead
  rmw_guard_condition_t * guard_condition_handle = rmw_guard_condition_allocate();
  guard_condition_handle->implementation_identifier = eclipse_zenoh_identifier;
  guard_condition_handle->data = new GuardCondition();

  return guard_condition_handle;
}

/// DESTROY GUARD CONDITION ====================================================
// Finalize a given guard condition handle, reclaim the resources, and deallocate the handle.
rmw_ret_t
rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition_handle)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "rmw_destroy_guard_condition");
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition_handle, RMW_RET_INVALID_ARGUMENT);

  if (guard_condition_handle->data) {
    delete static_cast<GuardCondition *>(guard_condition_handle->data);
  }
  rmw_guard_condition_free(guard_condition_handle);
  return RMW_RET_OK;
}

/// TRIGGER GUARD CONDITION ====================================================
rmw_ret_t
rmw_zenoh_common_trigger_guard_condition(
  const rmw_guard_condition_t * guard_condition_handle,
  const char * const eclipse_zenoh_identifier)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "rmw_trigger_guard_condition");

  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition_handle, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    guard_condition_handle,
    guard_condition_handle->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  static_cast<GuardCondition *>(guard_condition_handle->data)->trigger();

  return RMW_RET_OK;
}
