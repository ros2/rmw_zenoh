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

#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/types.h"

#include "detail/event.hpp"
#include "detail/graph_cache.hpp"
#include "detail/identifier.hpp"
#include "detail/rmw_data_types.hpp"


extern "C"
{
///=============================================================================
/// Initialize a rmw publisher event
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
  rmw_zenoh_cpp::rmw_publisher_data_t * pub_data =
    static_cast<rmw_zenoh_cpp::rmw_publisher_data_t *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data->entity, RMW_RET_INVALID_ARGUMENT);

  if (publisher->implementation_identifier != rmw_zenoh_cpp::rmw_zenoh_identifier) {
    RMW_SET_ERROR_MSG("Publisher implementation identifier not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  rmw_zenoh_cpp::rmw_zenoh_event_type_t zenoh_event_type =
    rmw_zenoh_cpp::zenoh_event_from_rmw_event(event_type);
  if (zenoh_event_type == rmw_zenoh_cpp::ZENOH_EVENT_INVALID) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "provided event_type %d is not supported by rmw_zenoh_cpp", event_type);
    return RMW_RET_UNSUPPORTED;
  }

  rmw_event->implementation_identifier = publisher->implementation_identifier;
  rmw_event->data = &pub_data->events_mgr;
  rmw_event->event_type = event_type;

  // Register the event with graph cache.
  pub_data->context->impl->graph_cache->set_qos_event_callback(
    pub_data->entity,
    zenoh_event_type,
    [pub_data,
    zenoh_event_type](std::unique_ptr<rmw_zenoh_cpp::rmw_zenoh_event_status_t> zenoh_event)
    {
      if (pub_data == nullptr) {
        return;
      }
      pub_data->events_mgr.add_new_event(
        zenoh_event_type,
        std::move(zenoh_event));
    }
  );

  return RMW_RET_OK;
}

///=============================================================================
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
  rmw_zenoh_cpp::rmw_subscription_data_t * sub_data =
    static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data->entity, RMW_RET_INVALID_ARGUMENT);

  if (subscription->implementation_identifier != rmw_zenoh_cpp::rmw_zenoh_identifier) {
    RMW_SET_ERROR_MSG(
      "Subscription implementation identifier not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  rmw_zenoh_cpp::rmw_zenoh_event_type_t zenoh_event_type =
    rmw_zenoh_cpp::zenoh_event_from_rmw_event(event_type);
  if (zenoh_event_type == rmw_zenoh_cpp::ZENOH_EVENT_INVALID) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "provided event_type %d is not supported by rmw_zenoh_cpp", event_type);
    return RMW_RET_UNSUPPORTED;
  }

  rmw_event->implementation_identifier = subscription->implementation_identifier;
  rmw_event->data = &sub_data->events_mgr;
  rmw_event->event_type = event_type;

  // Register the event with graph cache if the event is not ZENOH_EVENT_MESSAGE_LOST
  // since this is checked for in the subscription callback.
  if (zenoh_event_type == rmw_zenoh_cpp::ZENOH_EVENT_MESSAGE_LOST) {
    return RMW_RET_OK;
  }

  sub_data->context->impl->graph_cache->set_qos_event_callback(
    sub_data->entity,
    zenoh_event_type,
    [sub_data,
    zenoh_event_type](std::unique_ptr<rmw_zenoh_cpp::rmw_zenoh_event_status_t> zenoh_event)
    {
      if (sub_data == nullptr) {
        return;
      }
      sub_data->events_mgr.add_new_event(
        zenoh_event_type,
        std::move(zenoh_event));
    }
  );

  return RMW_RET_OK;
}

//==============================================================================
/// Set the callback function for the event.
rmw_ret_t
rmw_event_set_callback(
  rmw_event_t * rmw_event,
  rmw_event_callback_t callback,
  const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event->data, RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_zenoh_event_type_t zenoh_event_type =
    rmw_zenoh_cpp::zenoh_event_from_rmw_event(rmw_event->event_type);
  if (zenoh_event_type == rmw_zenoh_cpp::ZENOH_EVENT_INVALID) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "RMW Zenoh does not support event [%d]",
      rmw_event->event_type);
    return RMW_RET_ERROR;
  }

  // Both rmw_subscription_data_t and rmw_publisher_data_t inherit EventsBase.
  rmw_zenoh_cpp::EventsManager * event_data =
    static_cast<rmw_zenoh_cpp::EventsManager *>(rmw_event->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(event_data, RMW_RET_INVALID_ARGUMENT);
  event_data->event_set_callback(zenoh_event_type, callback, user_data);

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t
rmw_take_event(
  const rmw_event_t * event_handle,
  void * event_info,
  bool * taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(event_handle, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(event_info, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  *taken = false;

  if (event_handle->implementation_identifier != rmw_zenoh_cpp::rmw_zenoh_identifier) {
    RMW_SET_ERROR_MSG(
      "Event implementation identifier not from this implementation");
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
  }

  rmw_zenoh_cpp::rmw_zenoh_event_type_t zenoh_event_type =
    rmw_zenoh_cpp::zenoh_event_from_rmw_event(event_handle->event_type);
  if (zenoh_event_type == rmw_zenoh_cpp::ZENOH_EVENT_INVALID) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "RMW Zenoh does not support event [%d]",
      event_handle->event_type);
    return RMW_RET_ERROR;
  }

  rmw_zenoh_cpp::EventsManager * event_data =
    static_cast<rmw_zenoh_cpp::EventsManager *>(event_handle->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(event_data, RMW_RET_INVALID_ARGUMENT);
  std::unique_ptr<rmw_zenoh_cpp::rmw_zenoh_event_status_t> st = event_data->pop_next_event(
    zenoh_event_type);
  if (st == nullptr) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "rmw_take_event called when event queue for event type [%d] is empty",
      event_handle->event_type);
    return RMW_RET_ERROR;
  }

  // Now depending on the event, populate the rmw event status.
  switch (zenoh_event_type) {
    case rmw_zenoh_cpp::ZENOH_EVENT_REQUESTED_QOS_INCOMPATIBLE: {
        auto ei = static_cast<rmw_requested_qos_incompatible_event_status_t *>(event_info);
        RMW_CHECK_ARGUMENT_FOR_NULL(ei, RMW_RET_INVALID_ARGUMENT);
        ei->total_count = st->total_count;
        ei->total_count_change = st->total_count_change;
        *taken = true;
        return RMW_RET_OK;
      }
    case rmw_zenoh_cpp::ZENOH_EVENT_MESSAGE_LOST: {
        auto ei = static_cast<rmw_message_lost_status_t *>(event_info);
        RMW_CHECK_ARGUMENT_FOR_NULL(ei, RMW_RET_INVALID_ARGUMENT);
        ei->total_count = st->total_count;
        ei->total_count_change = st->total_count_change;
        *taken = true;
        return RMW_RET_OK;
      }
    case rmw_zenoh_cpp::ZENOH_EVENT_PUBLICATION_MATCHED:
    case rmw_zenoh_cpp::ZENOH_EVENT_SUBSCRIPTION_MATCHED: {
        auto ei = static_cast<rmw_matched_status_t *>(event_info);
        RMW_CHECK_ARGUMENT_FOR_NULL(ei, RMW_RET_INVALID_ARGUMENT);
        ei->total_count = st->total_count;
        ei->total_count_change = st->total_count_change;
        ei->current_count = st->current_count;
        ei->current_count_change = st->current_count_change;
        *taken = true;
        return RMW_RET_OK;
      }
    case rmw_zenoh_cpp::ZENOH_EVENT_OFFERED_QOS_INCOMPATIBLE: {
        auto ei = static_cast<rmw_offered_qos_incompatible_event_status_t *>(event_info);
        RMW_CHECK_ARGUMENT_FOR_NULL(ei, RMW_RET_INVALID_ARGUMENT);
        ei->total_count = st->total_count;
        ei->total_count_change = st->total_count_change;
        *taken = true;
        return RMW_RET_OK;
      }
    default: {
        return RMW_RET_INVALID_ARGUMENT;
      }
  }

  return RMW_RET_ERROR;
}
}  // extern "C"
