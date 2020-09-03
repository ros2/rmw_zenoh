#include "wait_impl.hpp"

#include "rcutils/logging_macros.h"
#include "service_impl.hpp"
#include "client_impl.hpp"
#include "pubsub_impl.hpp"

/// HELPER FUNCTION FOR WAIT ===================================================
bool check_wait_conditions(
  const rmw_subscriptions_t * subscriptions,
  const rmw_guard_conditions_t * guard_conditions,
  const rmw_services_t * services,
  const rmw_clients_t * clients,
  const rmw_events_t * events,
  bool finalize)
{
  // NOTE(CH3): On the finalize parameter
  // This check function is used as a predicate to wait on a condition variable. But rcl expects
  // rmw to set any passed in pointers to NULL if the condition is not ready.
  //
  // The finalize parameter is used to make sure that this setting to NULL only happens ONCE per
  // rmw_wait call. Otherwise on repeat calls to check the predicate, things will break since
  // it'll try to compare or dereference a nullptr.

  bool stop_wait = false;

  // SUBSCRIPTIONS =============================================================
  if (subscriptions) {
    size_t subscriptions_ready = 0;

    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
        auto subscription_data = static_cast<rmw_subscription_data_t *>(
          subscriptions->subscribers[i]);
        if (subscription_data->zn_message_queue_.empty()) {
          if (finalize) {
            // Setting to nullptr lets rcl know that this subscription is not ready
            subscriptions->subscribers[i] = nullptr;
          }
        } else {
          subscriptions_ready++;
          stop_wait = true;
        }
    }

    if (finalize) {
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "[rmw_wait] SUBSCRIPTIONS READY: %ld",
        subscriptions_ready);
    }
  }

  // SERVICES ==================================================================
  if (services) {
    size_t services_ready = 0;

    for (size_t i = 0; i < services->service_count; ++i) {
        auto service_data = static_cast<rmw_service_data_t *>(
          services->services[i]);
        if (service_data->zn_request_message_queue_.empty()) {
          if (finalize) {
            // Setting to nullptr lets rcl know that this service is not ready
            services->services[i] = nullptr;
          }
        } else {
          services_ready++;
          stop_wait = true;
        }
    }

    if (finalize) {
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "[rmw_wait] SERVICES READY: %ld",
        services_ready);
    }
  }

  return stop_wait;

  // CLIENTS ===================================================================
  if (!rmw_client_data_t::zn_response_messages_.empty()) {
    RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "[rmw_wait] RESPONSE MESSAGES IN QUEUE: %ld",
                            rmw_client_data_t::zn_response_messages_.size());
    return true;
  }

  // Clients: If there are service server availabiity messages ready, continue
  if (!rmw_client_data_t::zn_availability_query_responses_.empty()) {
    RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "[rmw_wait] AVAILABILITY QUERY RESPONSES: %ld",
                            rmw_client_data_t::zn_availability_query_responses_.size());
    return true;
  }

  // TODO(CH3): Handle events
  // if (events) {
  //   for (size_t i = 0; i < events->event_count; ++i) {
  //     auto event = static_cast<rmw_event_t *>(events->events[i]);
  //     auto custom_event_info = static_cast<rmw_publisher_data_t *>(event->data);
  //     if (custom_event_info->getListener()->hasEvent(event->event_type)) {
  //       return true;
  //     }
  //   }
  // }

  // TODO(CH3): Handle guard conditions
  // For now we'll wait because they keep getting triggered by the client libraries.
  // if (guard_conditions) {
  //   for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
  //     void * data = guard_conditions->guard_conditions[i];
  //     auto guard_condition = static_cast<GuardCondition *>(data);
  //     if (guard_condition && guard_condition->hasTriggered()) {
  //       RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "[rmw_wait] GUARD CONDITION TRIGGERED");
  //       return true;
  //     }
  //   }
  // }

  return false;
}
