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

// Docs: http://docs.ros2.org/latest/api/rmw/event_8h.html

#include "rcutils/logging_macros.h"

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/event.h"

#include "rmw_zenoh_cpp/identifier.hpp"

extern "C"
{
// STUB
rmw_ret_t
rmw_take_event(const rmw_event_t * event_handle, void * event_info, bool * taken)
{
  (void)event_info;

  // Because we are currently not (intentionally) requesting any events, this
  // message is a warning to future-us that we aren't expecting to be here!
  RCUTILS_LOG_WARN_NAMED("rmw_zenoh_cpp", "rmw_take_event() WOAH");

  switch (event_handle->event_type) {
    case RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE:
      RCUTILS_LOG_WARN_NAMED("rmw_zenoh_cpp", "rmw_take_event(): requested qos incompatible");
      break;

    case RMW_EVENT_OFFERED_QOS_INCOMPATIBLE:
      RCUTILS_LOG_WARN_NAMED("rmw_zenoh_cpp", "rmw_take_event(): offered qos incompatible");
      break;

    default:
      break;
  }

  *taken = true;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_event_init(
  rmw_event_t * event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_publisher_event_init");

  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(event, RMW_RET_INVALID_ARGUMENT);

  if (event_type == RMW_EVENT_INVALID) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  if (event_type == RMW_EVENT_OFFERED_QOS_INCOMPATIBLE)
  {
    // Because rmw_zenoh_cpp does not currently have multiple QoS, we will not be handling this event.
    return RMW_RET_UNSUPPORTED;
  }

  RCUTILS_LOG_ERROR_NAMED("rmw_zenoh_cpp", "accepting rmw_subscriber_event_init() for unhandled event!");
  event->implementation_identifier = publisher->implementation_identifier;
  event->data = publisher->data;
  event->event_type = event_type;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_event_init(
  rmw_event_t * event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_subscription_event_init");

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(event, RMW_RET_INVALID_ARGUMENT);

  if (event_type == RMW_EVENT_INVALID) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );

  if (event_type == RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE)
  {
    // Because rmw_zenoh_cpp does not currently have multiple QoS, we will not be handling this event.
    return RMW_RET_UNSUPPORTED;
  }

  RCUTILS_LOG_ERROR_NAMED("rmw_zenoh_cpp", "accepting rmw_subscriber_event_init() for unhandled event!");
  event->implementation_identifier = subscription->implementation_identifier;
  event->data = subscription->data;
  event->event_type = event_type;

  return RMW_RET_OK;
}
}  // extern "C"
