// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <unordered_map>

#include "rmw/error_handling.h"
#include "rmw/event.h"

#include "detail/identifier.hpp"

extern "C"
{
static const std::unordered_map<rmw_event_type_t, uint32_t> event_map{
  // TODO(clalancette): Implement some events
};

///==============================================================================
/// Initialize a rmw subscription event
rmw_ret_t
rmw_publisher_event_init(
  rmw_event_t * rmw_event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher->implementation_identifier, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher->data, RMW_RET_INVALID_ARGUMENT);

  if (publisher->implementation_identifier != rmw_zenoh_identifier) {
    RMW_SET_ERROR_MSG("Publisher implementation identifier not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (event_map.count(event_type) != 1) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "provided event_type %d is not supported by rmw_zenoh_cpp", event_type);
    return RMW_RET_UNSUPPORTED;
  }

  rmw_event->implementation_identifier = publisher->implementation_identifier;
  rmw_event->data = publisher->data;
  rmw_event->event_type = event_type;

  return RMW_RET_OK;
}

///==============================================================================
/// Take an event from the event handle.
rmw_ret_t
rmw_subscription_event_init(
  rmw_event_t * rmw_event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->implementation_identifier, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_INVALID_ARGUMENT);

  if (subscription->implementation_identifier != rmw_zenoh_identifier) {
    RMW_SET_ERROR_MSG(
      "Subscription implementation identifier not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  if (event_map.count(event_type) != 1) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "provided event_type %d is not supported by rmw_zenoh_cpp", event_type);
    return RMW_RET_UNSUPPORTED;
  }

  rmw_event->implementation_identifier = subscription->implementation_identifier;
  rmw_event->data = subscription->data;
  rmw_event->event_type = event_type;

  return RMW_RET_OK;
}

///==============================================================================
rmw_ret_t
rmw_take_event(
  const rmw_event_t * event_handle,
  void * event_info,
  bool * taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(event_handle, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(event_info, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  if (event_handle->implementation_identifier != rmw_zenoh_identifier) {
    RMW_SET_ERROR_MSG(
      "Event implementation identifier not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  switch (event_handle->event_type) {
    case RMW_EVENT_INVALID:
      break;
    default:
      break;
  }

  *taken = false;
  return RMW_RET_ERROR;
}
}  // extern "C"
