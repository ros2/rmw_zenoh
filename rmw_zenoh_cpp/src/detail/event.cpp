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

#include <deque>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>

#include "event.hpp"

#include "rcutils/logging_macros.h"

#include "rmw/error_handling.h"

namespace
{
// RMW Event types that we support in rmw_zenoh.
static const std::unordered_map<rmw_event_type_t, rmw_zenoh_cpp::rmw_zenoh_event_type_t> event_map{
  {RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE, rmw_zenoh_cpp::ZENOH_EVENT_REQUESTED_QOS_INCOMPATIBLE},
  {RMW_EVENT_OFFERED_QOS_INCOMPATIBLE, rmw_zenoh_cpp::ZENOH_EVENT_OFFERED_QOS_INCOMPATIBLE},
  {RMW_EVENT_MESSAGE_LOST, rmw_zenoh_cpp::ZENOH_EVENT_MESSAGE_LOST},
  {RMW_EVENT_SUBSCRIPTION_MATCHED, rmw_zenoh_cpp::ZENOH_EVENT_SUBSCRIPTION_MATCHED},
  {RMW_EVENT_PUBLICATION_MATCHED, rmw_zenoh_cpp::ZENOH_EVENT_PUBLICATION_MATCHED},
  {RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE,
    rmw_zenoh_cpp::ZENOH_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE},
  {RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE, rmw_zenoh_cpp::ZENOH_EVENT_PUBLISHER_INCOMPATIBLE_TYPE}
  // TODO(clalancette): Implement remaining events
};
}  // namespace

namespace rmw_zenoh_cpp
{
rmw_zenoh_event_type_t zenoh_event_from_rmw_event(rmw_event_type_t rmw_event_type)
{
  auto zenoh_event_it = event_map.find(rmw_event_type);
  if (zenoh_event_it != event_map.end()) {
    return zenoh_event_it->second;
  }

  return ZENOH_EVENT_INVALID;
}

///=============================================================================
void DataCallbackManager::set_callback(
  const void * user_data, rmw_event_callback_t callback)
{
  std::lock_guard<std::mutex> lock_mutex(event_mutex_);

  if (callback) {
    // Push events arrived before setting the the executor callback.
    if (unread_count_) {
      callback(user_data, unread_count_);
      unread_count_ = 0;
    }
    user_data_ = user_data;
    callback_ = callback;
  } else {
    user_data_ = nullptr;
    callback_ = nullptr;
  }
}

///=============================================================================
void DataCallbackManager::trigger_callback()
{
  // Trigger the user provided event callback if available.
  std::lock_guard<std::mutex> lock_mutex(event_mutex_);
  if (callback_ != nullptr) {
    callback_(user_data_, 1);
  } else {
    ++unread_count_;
  }
}

///=============================================================================
void EventsManager::event_set_callback(
  rmw_zenoh_event_type_t event_id,
  rmw_event_callback_t callback,
  const void * user_data)
{
  if (event_id > ZENOH_EVENT_ID_MAX) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "RMW Zenoh is not correctly configured to handle rmw_zenoh_event_type_t [%d]. "
      "Report this bug.",
      event_id);
    return;
  }

  std::lock_guard<std::mutex> lock(event_mutex_);

  // Set the user callback data
  event_callback_[event_id] = callback;
  event_data_[event_id] = user_data;

  if (callback && event_unread_count_[event_id]) {
    // Push events happened before having assigned a callback
    callback(user_data, event_unread_count_[event_id]);
    event_unread_count_[event_id] = 0;
  }
  return;
}

///=============================================================================
void EventsManager::trigger_event_callback(rmw_zenoh_event_type_t event_id)
{
  if (event_id > ZENOH_EVENT_ID_MAX) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "RMW Zenoh is not correctly configured to handle rmw_zenoh_event_type_t [%d]. "
      "Report this bug.",
      event_id);
    return;
  }

  std::lock_guard<std::mutex> lock(event_mutex_);

  if (event_callback_[event_id] != nullptr) {
    event_callback_[event_id](event_data_[event_id], 1);
  } else {
    ++event_unread_count_[event_id];
  }
  return;
}

///=============================================================================
std::unique_ptr<rmw_zenoh_event_status_t> EventsManager::pop_next_event(
  rmw_zenoh_event_type_t event_id)
{
  if (event_id > ZENOH_EVENT_ID_MAX) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "RMW Zenoh is not correctly configured to handle rmw_zenoh_event_type_t [%d]. "
      "Report this bug.",
      event_id);
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(event_mutex_);

  if (event_queues_[event_id].empty()) {
    // This tells rcl that the check for a new events was done, but no events have come in yet.
    return nullptr;
  }

  std::unique_ptr<rmw_zenoh_event_status_t> event_status =
    std::move(event_queues_[event_id].front());
  event_queues_[event_id].pop_front();

  return event_status;
}

///=============================================================================
void EventsManager::add_new_event(
  rmw_zenoh_event_type_t event_id,
  std::unique_ptr<rmw_zenoh_event_status_t> event)
{
  if (event_id > ZENOH_EVENT_ID_MAX) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "RMW Zenoh is not correctly configured to handle rmw_zenoh_event_type_t [%d]. "
      "Report this bug.",
      event_id);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(event_mutex_);

    std::deque<std::unique_ptr<rmw_zenoh_event_status_t>> & event_queue = event_queues_[event_id];
    if (event_queue.size() >= event_queue_depth_) {
      // Log warning if message is discarded due to hitting the queue depth
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp",
        "Event queue depth of %ld reached, discarding oldest message "
        "for event type %d",
        event_queue_depth_,
        event_id);

      event_queue.pop_front();
    }

    event_queue.emplace_back(std::move(event));
  }

  // Since we added new data, trigger event callback and guard condition if they are available
  trigger_event_callback(event_id);
  notify_event(event_id);
}

///=============================================================================
bool EventsManager::queue_has_data_and_attach_condition_if_not(
  rmw_zenoh_event_type_t event_id,
  rmw_wait_set_data_t * wait_set_data)
{
  if (event_id > ZENOH_EVENT_ID_MAX) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "RMW Zenoh is not correctly configured to handle rmw_zenoh_event_type_t [%d]. "
      "Report this bug.",
      event_id);
    return false;
  }

  std::lock_guard<std::mutex> lock(event_condition_mutex_);

  if (!event_queues_[event_id].empty()) {
    return true;
  }

  wait_set_data_[event_id] = wait_set_data;

  return false;
}

///=============================================================================
bool EventsManager::detach_condition_and_event_queue_is_empty(rmw_zenoh_event_type_t event_id)
{
  if (event_id > ZENOH_EVENT_ID_MAX) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "RMW Zenoh is not correctly configured to handle rmw_zenoh_event_type_t [%d]. "
      "Report this bug.",
      event_id);
    return true;
  }

  std::lock_guard<std::mutex> lock(event_condition_mutex_);

  wait_set_data_[event_id] = nullptr;

  return event_queues_[event_id].empty();
}

///=============================================================================
void EventsManager::notify_event(rmw_zenoh_event_type_t event_id)
{
  if (event_id > ZENOH_EVENT_ID_MAX) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "RMW Zenoh is not correctly configured to handle rmw_zenoh_event_type_t [%d]. "
      "Report this bug.",
      event_id);
    return;
  }

  std::lock_guard<std::mutex> lock(event_condition_mutex_);
  if (wait_set_data_[event_id] != nullptr) {
    std::lock_guard<std::mutex> wait_set_lock(wait_set_data_[event_id]->condition_mutex);
    wait_set_data_[event_id]->triggered = true;
    wait_set_data_[event_id]->condition_variable.notify_one();
  }
}
}  // namespace rmw_zenoh_cpp
