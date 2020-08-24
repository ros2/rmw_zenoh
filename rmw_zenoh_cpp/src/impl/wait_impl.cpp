#include "rcutils/logging_macros.h"

#include "wait_impl.hpp"
#include "pubsub_impl.hpp"
#include "service_impl.hpp"
#include "client_impl.hpp"

/// HELPER FUNCTION FOR WAIT ===================================================
bool check_wait_conditions(
  const rmw_subscriptions_t * subscriptions,
  const rmw_guard_conditions_t * guard_conditions,
  const rmw_services_t * services,
  const rmw_clients_t * clients,
  const rmw_events_t * events)
{
  // Subscriptions: If there are subscription messages ready, continue
  if (!rmw_subscription_data_t::zn_messages_.empty()) {
    // RCUTILS_LOG_INFO_NAMED(
    //   "rmw_zenoh_cpp",
    //   "[rmw_wait] SUBSCRIPTION MESSAGES IN QUEUE: %ld", rmw_subscription_data_t::zn_messages_.size()
    // );
    return true;
  }

  // Services: If there are request messages ready, continue
  if (!rmw_service_data_t::zn_request_messages_.empty()) {
    // RCUTILS_LOG_INFO_NAMED(
    //   "rmw_zenoh_cpp",
    //   "[rmw_wait] REQUEST MESSAGES IN QUEUE: %ld", rmw_service_data_t::zn_request_messages_.size()
    // );
    return true;
  }

  // Clients: If there are response messages ready, continue
  if (!rmw_client_data_t::zn_response_messages_.empty()) {
    // RCUTILS_LOG_INFO_NAMED(
    //   "rmw_zenoh_cpp",
    //   "[rmw_wait] RESPONSE MESSAGES IN QUEUE: %ld", rmw_client_data_t::zn_response_messages_.size()
    // );
    return true;
  }

  // Clients: If there are service server availabiity messages ready, continue
  if (!rmw_client_data_t::zn_availability_query_responses_.empty()) {
    // RCUTILS_LOG_INFO_NAMED(
    //   "rmw_zenoh_cpp",
    //   "[rmw_wait] RESPONSE MESSAGES IN QUEUE: %ld", rmw_client_data_t::zn_availability_query_responses_.size()
    // );
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
