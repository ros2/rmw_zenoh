// Doc: http://docs.ros2.org/latest/api/rmw/rmw_8h.html
// Edited from: https://github.com/ros2/rmw_fastrtps/blob/master/rmw_fastrtps_shared_cpp/src/rmw_wait_set.cpp
// Under the Apache 2.0 license

#include "rcutils/logging_macros.h"

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/event.h"

#include "rmw_zenoh_cpp/identifier.hpp"
#include "types/wait_set_data.hpp"

extern "C"
{

rmw_wait_set_t *
rmw_create_wait_set(rmw_context_t * context, size_t max_conditions)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr
  );

  (void)max_conditions;

  // NOTE(CH3): Unfortunately we can't do custom allocation here because the destruction method
  // does not pass in a context from which we can draw an allocator from
  rmw_wait_set_t * wait_set = rmw_wait_set_allocate();
  rmw_wait_set_data_t * wait_set_data = nullptr;

  // From here onward, error results in unrolling in the goto fail block.
  if (!wait_set) {
    RMW_SET_ERROR_MSG("failed to allocate wait set");
    goto fail;
  }
  wait_set->implementation_identifier = eclipse_zenoh_identifier;
  wait_set->data = rmw_allocate(sizeof(rmw_wait_set_data_t));
  // This should default-construct the fields of rmw_wait_set_data_t
  wait_set_data = static_cast<rmw_wait_set_data_t *>(wait_set->data);
  // cppcheck-suppress syntaxError
  RMW_TRY_PLACEMENT_NEW(wait_set_data, wait_set_data, goto fail, rmw_wait_set_data_t, )
  if (!wait_set_data) {
    RMW_SET_ERROR_MSG("failed to construct wait set info struct");
    goto fail;
  }

  return wait_set;

fail:
  if (wait_set) {
    if (wait_set->data) {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        wait_set_data->~rmw_wait_set_data_t(), wait_set_data)
      rmw_free(wait_set->data);
    }
    rmw_wait_set_free(wait_set);
  }
  return nullptr;
}

rmw_ret_t
rmw_destroy_wait_set(rmw_wait_set_t * wait_set)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    wait_set,
    wait_set->implementation_identifier, eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION)

  auto result = RMW_RET_OK;
  auto wait_set_data = static_cast<rmw_wait_set_data_t *>(wait_set->data);
  if (!wait_set_data) {
    RMW_SET_ERROR_MSG("wait set info is null");
    return RMW_RET_ERROR;
  }
  std::mutex * conditionMutex = &wait_set_data->condition_mutex;
  if (!conditionMutex) {
    RMW_SET_ERROR_MSG("wait set mutex is null");
    return RMW_RET_ERROR;
  }

  if (wait_set->data) {
    if (wait_set_data) {
      RMW_TRY_DESTRUCTOR(
        wait_set_data->~rmw_wait_set_data_t(), wait_set_data, result = RMW_RET_ERROR)
    }
    rmw_free(wait_set->data);
  }
  rmw_wait_set_free(wait_set);
  return result;
}

// STUB
rmw_ret_t
rmw_wait(
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  (void)subscriptions;
  (void)guard_conditions;
  (void)services;
  (void)clients;
  (void)events;
  (void)wait_set;
  (void)wait_timeout;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_wait (STUB)");
  // return RMW_RET_ERROR;

  return RMW_RET_OK;
}

} // extern "C"
