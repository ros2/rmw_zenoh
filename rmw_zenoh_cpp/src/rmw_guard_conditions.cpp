// Doc: http://docs.ros2.org/latest/api/rmw/rmw_8h.html

#include "rcutils/logging_macros.h"

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/event.h"

#include "rmw_zenoh_cpp/identifier.hpp"
#include "types/guard_condition.hpp"

extern "C"
{
/// CREATE GUARD CONDITION =====================================================
// Create a guard condition and return a handle to that guard condition.
rmw_guard_condition_t *
rmw_create_guard_condition(rmw_context_t * context)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr
  );

  // NOTE(CH3): Unfortunately we can't do custom allocation here because the destruction method
  // does not pass in a context from which we can draw an allocator from
  rmw_guard_condition_t * guard_condition_handle = new rmw_guard_condition_t;
  guard_condition_handle->implementation_identifier = eclipse_zenoh_identifier;
  guard_condition_handle->data = new GuardCondition();

  return guard_condition_handle;
}

/// DESTROY GUARD CONDITION ====================================================
// Finalize a given guard condition handle, reclaim the resources, and deallocate the handle.
rmw_ret_t
rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition_handle)
{
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
  assert(guard_condition_handle);

  if (guard_condition_handle->implementation_identifier != eclipse_zenoh_identifier) {
    RMW_SET_ERROR_MSG("guard condition handle not from this implementation");
    return RMW_RET_ERROR;
  }

  auto guard_condition = static_cast<GuardCondition *>(guard_condition_handle->data);
  guard_condition->trigger();

  return RMW_RET_OK;
}

/// UNIMPLEMENTED ==============================================================
const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(const rmw_node_t * node)
{
  (void)node;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_node_get_graph_guard_condition");
  return nullptr;
}

} // extern "C"
