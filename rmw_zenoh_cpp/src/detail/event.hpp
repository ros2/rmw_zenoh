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

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rmw/event.h"
#include "rmw/event_callback_type.h"


///=============================================================================
// A struct that represents an event status in rmw_zenoh.
enum rmw_zenoh_event_type_t
{
  // sentinel value
  ZENOH_EVENT_INVALID,

  // subscription events
  ZENOH_EVENT_REQUESTED_QOS_INCOMPATIBLE,
  ZENOH_EVENT_MESSAGE_LOST,
  // RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE,
  ZENOH_EVENT_SUBSCRIPTION_MATCHED,

  // publisher events
  // RMW_EVENT_LIVELINESS_LOST,
  // RMW_EVENT_OFFERED_DEADLINE_MISSED,
  ZENOH_EVENT_OFFERED_QOS_INCOMPATIBLE,
  // RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE,
  ZENOH_EVENT_PUBLICATION_MATCHED,
};

/// Helper value to indicate the maximum index of events supported.
#define ZENOH_EVENT_ID_MAX rmw_zenoh_event_type_t::ZENOH_EVENT_PUBLICATION_MATCHED

// RMW Event types that we support in rmw_zenoh.
static const std::unordered_map<rmw_event_type_t, rmw_zenoh_event_type_t> event_map{
  {RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE, ZENOH_EVENT_REQUESTED_QOS_INCOMPATIBLE},
  {RMW_EVENT_OFFERED_QOS_INCOMPATIBLE, ZENOH_EVENT_OFFERED_QOS_INCOMPATIBLE},
  {RMW_EVENT_MESSAGE_LOST, ZENOH_EVENT_MESSAGE_LOST},
  {RMW_EVENT_SUBSCRIPTION_MATCHED, ZENOH_EVENT_SUBSCRIPTION_MATCHED},
  {RMW_EVENT_PUBLICATION_MATCHED, ZENOH_EVENT_PUBLICATION_MATCHED}
  // TODO(clalancette): Implement remaining events
};

///=============================================================================
/// A struct to store status changes which can be mapped to rmw event statuses.
/// The data field can be used to store serialized information for more complex statuses.
struct rmw_zenoh_event_status_t
{
  size_t total_count;
  size_t total_count_change;
  size_t current_count;
  size_t current_count_change;
  std::string data;
};

///=============================================================================
/// Base class to be inherited by entities that support events.
class EventsBase
{
public:
  /// @brief Set the user defined callback that should be called when
  /// a new message/response/request is received.
  /// @param user_data the data that should be passed to the callback.
  /// @param callback the callback to be set.
  void set_user_callback(const void * user_data, rmw_event_callback_t callback);

  /// Trigger the user callback.
  void trigger_user_callback();

  /// @brief  Set the callback to be triggered when the relevant event is triggered.
  /// @param event_id the id of the event
  /// @param callback the callback to trigger for this event.
  /// @param user_data the data to be passed to the callback.
  void event_set_callback(
    rmw_zenoh_event_type_t event_id,
    rmw_event_callback_t callback,
    const void * user_data);

  /// @brief  Returns true if the event queue is empty.
  /// @param event_id the event id whose event queue should be checked.
  bool event_queue_is_empty(rmw_zenoh_event_type_t event_id) const;

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
  void attach_event_condition(
    rmw_zenoh_event_type_t event_id,
    std::condition_variable * condition_variable);

  /// @brief Detach the condition variable provided by rmw_wait.
  void detach_event_condition(rmw_zenoh_event_type_t event_id);

private:
  /// @brief Trigger the callback for an event.
  /// @param event_id the event id whose callback should be triggered.
  void trigger_event_callback(rmw_zenoh_event_type_t event_id);

  /// Notify once event is added to an event queue.
  void notify_event(rmw_zenoh_event_type_t event_id);

  /// Mutex to lock when read/writing members.
  // The mutex is recursive as add_new_event() invokes `trigger_event_callback()`.
  mutable std::recursive_mutex event_mutex_;
  /// Mutex to lock for event_condition.
  mutable std::mutex event_condition_mutex_;
  /// Condition variable to attach for event notifications.
  std::condition_variable * event_conditions_[ZENOH_EVENT_ID_MAX + 1]{nullptr};
  /// User callback that can be set via set_user_callback().
  rmw_event_callback_t callback_ {nullptr};
  /// User data that should be passed to the user callback.
  const void * user_data_ {nullptr};
  /// Count for
  size_t unread_count_ {0};
  rmw_event_callback_t event_callback_[ZENOH_EVENT_ID_MAX + 1] {nullptr};
  const void * event_data_[ZENOH_EVENT_ID_MAX + 1] {nullptr};
  size_t event_unread_count_[ZENOH_EVENT_ID_MAX + 1] {0};
  // A dequeue of events for each type of event this RMW supports.
  std::deque<std::unique_ptr<rmw_zenoh_event_status_t>> event_queues_[ZENOH_EVENT_ID_MAX + 1] {};
  const std::size_t event_queue_depth_ = 10;
};

#endif  // DETAIL__EVENT_HPP_
