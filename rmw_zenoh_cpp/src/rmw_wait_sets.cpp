// Doc: http://docs.ros2.org/latest/api/rmw/rmw_8h.html
// Edited from: https://github.com/ros2/rmw_fastrtps/blob/master/rmw_fastrtps_shared_cpp/src/rmw_wait_set.cpp
// Under the Apache 2.0 license

#include "rcutils/logging_macros.h"

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/event.h"

#include "rmw_zenoh_cpp/identifier.hpp"
#include "impl/wait_impl.hpp"

extern "C"
{
/// CREATE WAIT SET ============================================================
// Create and return a wait set to wait to store conditions that the middleware will block on
rmw_wait_set_t *
rmw_create_wait_set(rmw_context_t * context, size_t max_conditions)
{
  (void)max_conditions;

  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_create_wait_set");

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(context,
                                   context->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return nullptr);

  // NOTE(CH3): Unfortunately we can't do custom allocation here because the destruction method
  // does not pass in a context from which we can draw an allocator from
  // TODO(CH3): Once https://github.com/ros2/rmw/issues/260 is addressed, use an allocator
  rmw_wait_set_t * wait_set = rmw_wait_set_allocate();

  // From here onward, error results in unrolling in the goto fail block.
  if (!wait_set) {
    RMW_SET_ERROR_MSG("failed to allocate wait set");
    goto fail;
  }
  wait_set->implementation_identifier = eclipse_zenoh_identifier;
  wait_set->data = rmw_allocate(sizeof(rmw_wait_set_data_t));

  // Invoke placement new
  new(wait_set->data) rmw_wait_set_data_t;
  if (!wait_set->data) {
    RMW_SET_ERROR_MSG("failed to construct wait set info struct");
    goto fail;
  }

  return wait_set;

fail:
  if (wait_set) {
    if (wait_set->data) {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(static_cast<rmw_wait_set_data_t *>(wait_set->data)
          ->~rmw_wait_set_data_t(), wait_set->data);
      rmw_free(wait_set->data);
    }
    rmw_wait_set_free(wait_set);
  }
  return nullptr;
}

/// DESTROY WAIT SET ===========================================================
// Destroy and deallocate a wait set
rmw_ret_t
rmw_destroy_wait_set(rmw_wait_set_t * wait_set)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_destroy_wait_set");

  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(wait_set,
                                   wait_set->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

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
          wait_set_data->~rmw_wait_set_data_t(), wait_set_data, result = RMW_RET_ERROR);
    }
    rmw_free(wait_set->data);
  }

  rmw_wait_set_free(wait_set);
  return result;
}

/// WAIT =======================================================================
// Block until data arrives, or wait set conditions are fulfilled, or timeout
rmw_ret_t
rmw_wait(  // All parameters are in parameters
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);

  RCUTILS_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[rmw_wait] %ld subscriptions, %ld srv_servers, %ld srv_clients, %ld events, %ld guard conditions",
      subscriptions->subscriber_count,
      services->service_count,
      clients->client_count,
      events->event_count,
      guard_conditions->guard_condition_count);

  if (wait_timeout) {
    RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "[rmw_wait] TIMEOUT: %ld s %ld ns",
                            wait_timeout->sec,
                            wait_timeout->nsec);
  }

  // OBTAIN SYNCHRONIZATION OBJECTS ============================================
  auto wait_set_info = static_cast<rmw_wait_set_data_t *>(wait_set->data);
  if (!wait_set_info) {
    RMW_SET_ERROR_MSG("Waitset info struct is null");
    return RMW_RET_ERROR;
  }

  std::mutex * condition_mutex = &wait_set_info->condition_mutex;
  if (!condition_mutex) {
    RMW_SET_ERROR_MSG("Mutex for wait set was null");
    return RMW_RET_ERROR;
  }

  std::condition_variable * condition_variable = &wait_set_info->condition;
  if (!condition_variable) {
    RMW_SET_ERROR_MSG("Condition variable for wait set was null");
    return RMW_RET_ERROR;
  }

  // NOTE(CH3): We may need to pass the mutex to any callback functions used by Zenoh
  //
  // Unfortunately, since all the callbacks are static, this means if we want to, we will have
  // to create a static mutex in something like wait_impl.cpp, assign it in here, pass it in to
  // each static callback function, and then unassign it later on.

  // CHECK WAIT CONDITIONS =====================================================
  std::unique_lock<std::mutex> lock(*condition_mutex);

  bool ready = check_wait_conditions(subscriptions, guard_conditions, services, clients, events);
  auto predicate = [subscriptions, guard_conditions, services, clients, events]() {
    return check_wait_conditions(subscriptions, guard_conditions, services, clients, events);
  };

  bool timed_out = false;

  if (!ready) {
    if (!wait_timeout) {
      // TODO(CH3): Remove this magic number once stable. This is to slow things down so things are
      // visible with all the printouts flying everywhere.
      condition_variable->wait_for(lock, std::chrono::milliseconds(1500), predicate);
    } else if (wait_timeout->sec > 0 || wait_timeout->nsec > 0) {
      auto wait_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::seconds(wait_timeout->sec));
      wait_time += std::chrono::nanoseconds(wait_timeout->nsec);

      timed_out = !condition_variable->wait_for(lock, wait_time, predicate);
    } else {
      timed_out = true;
    }
  }

  lock.unlock();

  if (timed_out) {
    RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "[rmw_wait] TIMED OUT");
    return RMW_RET_TIMEOUT;
  } else {
    return RMW_RET_OK;
  }
}
}  // extern "C"
