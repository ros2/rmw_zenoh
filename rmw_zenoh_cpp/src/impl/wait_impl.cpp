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

    if (finalize && subscriptions_ready > 0) {
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "[rmw_wait] SUBSCRIPTIONS READY: %ld",
        subscriptions_ready);
    }
  }

  // SERVICES ==================================================================
  if (services) {
    size_t services_ready = 0;

    for (size_t i = 0; i < services->service_count; ++i) {
        auto service_data = static_cast<rmw_service_data_t *>(services->services[i]);
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

    if (finalize && services_ready > 0) {
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "[rmw_wait] SERVICES READY: %ld",
        services_ready);
    }
  }


  // CLIENTS ===================================================================
  if (clients) {
    size_t clients_ready = 0;

    for (size_t i = 0; i < clients->client_count; ++i) {
        auto client_data = static_cast<rmw_client_data_t *>(clients->clients[i]);
        if (client_data->zn_response_message_queue_.empty()
            && client_data->zn_availability_query_responses_.empty()) {
          if (finalize) {
            // Setting to nullptr lets rcl know that this client is not ready
            clients->clients[i] = nullptr;
          }
        } else {
          clients_ready++;
          stop_wait = true;
        }
    }

    if (finalize && clients_ready > 0) {
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "[rmw_wait] CLIENTS READY: %ld",
        clients_ready);
    }
  }

  // GUARD CONDITIONS ==========================================================
  // STUB: NULLIFY/IGNORE ALL GUARD CONDITIONS FOR NOW
  for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
    guard_conditions->guard_conditions[i] = nullptr;
  }

  // NOTE(CH3): TODO(CH3): Uncomment this block when finalizing the RMW implementation
  // IGNORE GUARD CONDITIONS FOR NOW (They'll keep getting triggered super often, which is good for
  // responsiveness but not so much for debugging)

  // if (guard_conditions) {
  //   size_t guard_conditions_ready = 0;
  //
  //   for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
  //       auto guard_condition = static_cast<GuardCondition *>(guard_conditions->guard_conditions[i]);
  //       if (guard_condition && guard_condition->hasTriggered()) {
  //         if (finalize) {
  //           // Setting to nullptr lets rcl know that this guard_condition is not ready
  //           guard_conditions->guard_conditions[i] = nullptr;
  //         }
  //       } else {
  //         guard_conditions_ready++;
  //         stop_wait = true;
  //       }
  //   }
  //
  //   if (finalize && guard_conditions_ready > 0) {
  //     RCUTILS_LOG_DEBUG_NAMED(
  //       "rmw_zenoh_cpp", "[rmw_wait] GUARD CONDITIONS READY: %ld",
  //       guard_conditions_ready);
  //   }
  // }

  // EVENTS ====================================================================
  // STUB: NULLIFY/IGNORE ALL EVENTS FOR NOW
  // Notably, this causes the QoS warning to be suppressed..
  // Since that arises from the taking of QoS events, which aren't supported by Zenoh at the moment
  for (size_t i = 0; i < events->event_count; ++i) {
    events->events[i] = nullptr;
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

  return stop_wait;
}
