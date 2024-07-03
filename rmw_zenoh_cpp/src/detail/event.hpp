// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#ifndef DETAIL__EVENT_HPP_
#define DETAIL__EVENT_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "rmw/event.h"
#include "rmw/event_callback_type.h"

#include "rmw_wait_set_data.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
// A struct that represents an event status in rmw_zenoh.
enum rmw_zenoh_event_type_t
{
  // sentinel value
  ZENOH_EVENT_INVALID,

  // unimplemented subscription events.
  ZENOH_EVENT_LIVELINESS_CHANGED,
  ZENOH_EVENT_REQUESTED_DEADLINE_MISSED,
  // implemented subscription events.
  ZENOH_EVENT_REQUESTED_QOS_INCOMPATIBLE,
  ZENOH_EVENT_MESSAGE_LOST,
  ZENOH_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE,
  ZENOH_EVENT_SUBSCRIPTION_MATCHED,

  // unimplemented publisher events.
  ZENOH_EVENT_LIVELINESS_LOST,
  ZENOH_EVENT_OFFERED_DEADLINE_MISSED,
  // implemented publisher events.
  ZENOH_EVENT_OFFERED_QOS_INCOMPATIBLE,
  ZENOH_EVENT_PUBLISHER_INCOMPATIBLE_TYPE,
  ZENOH_EVENT_PUBLICATION_MATCHED,
};

/// Helper value to indicate the maximum index of events supported.
#define ZENOH_EVENT_ID_MAX rmw_zenoh_event_type_t::ZENOH_EVENT_PUBLICATION_MATCHED

rmw_zenoh_event_type_t zenoh_event_from_rmw_event(rmw_event_type_t rmw_event_type);

///=============================================================================
/// A struct to store status changes which can be mapped to rmw event statuses.
struct rmw_zenoh_event_status_t
{
  size_t total_count;
  size_t total_count_change;
  size_t current_count;
  size_t current_count_change;
  // The data field can be used to store serialized information for more complex statuses.
  std::string data;

  rmw_zenoh_event_status_t()
  : total_count(0),
    total_count_change(0),
    current_count(0),
    current_count_change(0)
  {}
};

///=============================================================================
/// A class that manages callbacks that should be triggered when a new
/// message/request/response is received by an entity.
class DataCallbackManager
{
public:
  /// @brief Set the user defined callback that should be called when
  /// a new message/response/request is received.
  /// @param user_data the data that should be passed to the callback.
  /// @param callback the callback to be set.
  void set_callback(const void * user_data, rmw_event_callback_t callback);

  /// Trigger the user callback.
  void trigger_callback();

private:
  std::mutex event_mutex_;
  /// User callback that can be set via set_callback().
  rmw_event_callback_t callback_ {nullptr};
  /// User data that should be passed to the user callback.
  const void * user_data_ {nullptr};
  /// number of trigger requests made before the callback was set.
  size_t unread_count_ {0};
};

/// Base class to be inherited by entities that support events.
class EventsManager
{
public:
  /// @brief  Set the callback to be triggered when the relevant event is triggered.
  /// @param event_id the id of the event
  /// @param callback the callback to trigger for this event.
  /// @param user_data the data to be passed to the callback.
  void event_set_callback(
    rmw_zenoh_event_type_t event_id,
    rmw_event_callback_t callback,
    const void * user_data);

  /// Pop the next event in the queue.
  /// @param event_id the event id whose queue should be popped.
  std::unique_ptr<rmw_zenoh_event_status_t> pop_next_event(
    rmw_zenoh_event_type_t event_id);

  /// Add an event status for an event.
  /// @param event_id the event id queue to which the status should be added.
  void add_new_event(
    rmw_zenoh_event_type_t event_id,
    std::unique_ptr<rmw_zenoh_event_status_t> event);

  /// @brief Attach the condition variable provided by rmw_wait.
  /// @param condition_variable to attach.
  bool queue_has_data_and_attach_condition_if_not(
    rmw_zenoh_event_type_t event_id,
    rmw_wait_set_data_t * wait_set_data);

  /// @brief Detach the condition variable provided by rmw_wait.
  bool detach_condition_and_event_queue_is_empty(rmw_zenoh_event_type_t event_id);

private:
  /// @brief Trigger the callback for an event.
  /// @param event_id the event id whose callback should be triggered.
  void trigger_event_callback(rmw_zenoh_event_type_t event_id);

  /// Notify once event is added to an event queue.
  void notify_event(rmw_zenoh_event_type_t event_id);

  /// Mutex to lock when read/writing members.
  mutable std::mutex event_mutex_;
  /// Mutex to lock for event_condition.
  mutable std::mutex event_condition_mutex_;
  /// Condition variable to attach for event notifications.
  rmw_wait_set_data_t * wait_set_data_[ZENOH_EVENT_ID_MAX + 1]{nullptr};

  rmw_event_callback_t event_callback_[ZENOH_EVENT_ID_MAX + 1] {nullptr};
  const void * event_data_[ZENOH_EVENT_ID_MAX + 1] {nullptr};
  size_t event_unread_count_[ZENOH_EVENT_ID_MAX + 1] {0};
  // A dequeue of events for each type of event this RMW supports.
  std::deque<std::unique_ptr<rmw_zenoh_event_status_t>> event_queues_[ZENOH_EVENT_ID_MAX + 1] {};
  const std::size_t event_queue_depth_ = 10;
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__EVENT_HPP_
