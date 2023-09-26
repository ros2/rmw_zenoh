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

#include <zenoh.h>

#include <mutex>
#include <utility>

#include "rcutils/logging_macros.h"

#include "rmw_data_types.hpp"

///==============================================================================
void sub_data_handler(
  const z_sample_t * sample,
  void * data)
{
  z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);

  auto sub_data = static_cast<rmw_subscription_data_t *>(data);
  if (sub_data == nullptr) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain rmw_subscription_data_t from data for "
      "subscription for %s",
      z_loan(keystr)
    );
    return;
  }

  rcutils_allocator_t * allocator = &sub_data->context->options.allocator;

  uint8_t * cdr_buffer =
    static_cast<uint8_t *>(allocator->allocate(sample->payload.len, allocator->state));
  if (cdr_buffer == nullptr) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Failed to allocate memory for cdr_buffer for "
      "subscription for %s",
      z_loan(keystr)
    );
    return;
  }
  memcpy(cdr_buffer, sample->payload.start, sample->payload.len);

  {
    std::lock_guard<std::mutex> lock(sub_data->message_queue_mutex);

    if (sub_data->message_queue.size() >= sub_data->queue_depth) {
      // Log warning if message is discarded due to hitting the queue depth
      RCUTILS_LOG_WARN_NAMED(
        "rmw_zenoh_cpp",
        "Message queue depth of %ld reached, discarding oldest message "
        "for subscription for %s",
        sub_data->queue_depth,
        z_loan(keystr));

      std::pair<size_t, uint8_t *> old = sub_data->message_queue.back();
      allocator->deallocate(old.second, allocator->state);
      sub_data->message_queue.pop_back();
    }

    sub_data->message_queue.push_front(std::make_pair(sample->payload.len, cdr_buffer));

    // Since we added new data, trigger the guard condition if it is available
    std::lock_guard<std::mutex> internal_lock(sub_data->internal_mutex);
    if (sub_data->condition != nullptr) {
      sub_data->condition->notify_one();
    }
  }

  z_drop(z_move(keystr));
}
