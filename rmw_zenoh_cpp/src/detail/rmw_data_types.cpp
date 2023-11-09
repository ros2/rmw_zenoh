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

      std::unique_ptr<saved_msg_data> old = std::move(sub_data->message_queue.back());
      z_drop(&old->payload);
      sub_data->message_queue.pop_back();
    }

    sub_data->message_queue.emplace_front(
      std::make_unique<saved_msg_data>(
        zc_sample_payload_rcinc(sample),
        sample->timestamp.time, sample->timestamp.id.id));

    // Since we added new data, trigger the guard condition if it is available
    std::lock_guard<std::mutex> internal_lock(sub_data->internal_mutex);
    if (sub_data->condition != nullptr) {
      sub_data->condition->notify_one();
    }
  }

  z_drop(z_move(keystr));
}
