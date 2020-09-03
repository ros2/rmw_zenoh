// Doc: http://docs.ros2.org/latest/api/rmw/rmw_8h.html
// Edited from: https://github.com/ros2/rmw_fastrtps/blob/master/rmw_fastrtps_shared_cpp/src/rmw_guard_condition.cpp
// Under the Apache 2.0 license

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/event.h"

#include "rcutils/logging_macros.h"

#include "rmw_zenoh_cpp/identifier.hpp"
#include "impl/guard_condition_impl.hpp"

extern "C"
{
/// CREATE GUARD CONDITION =====================================================
// Create a guard condition and return a handle to that guard condition.
rmw_guard_condition_t *
rmw_create_guard_condition(rmw_context_t * context)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_create_guard_condition");

  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(context,
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
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_destroy_guard_condition");

  if (guard_condition_handle) {
    delete static_cast<GuardCondition *>(guard_condition_handle->data);
    delete guard_condition_handle;
    return RMW_RET_OK;
  }

  return RMW_RET_ERROR;
}

/// TRIGGER GUARD CONDITION ====================================================
rmw_ret_t
rmw_trigger_guard_condition(const rmw_guard_condition_t * guard_condition_handle)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_trigger_guard_condition");

  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition_handle, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(guard_condition_handle,
                                   guard_condition_handle->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  static_cast<GuardCondition *>(guard_condition_handle->data)->trigger();

  return RMW_RET_OK;
}
}  // extern "C"
