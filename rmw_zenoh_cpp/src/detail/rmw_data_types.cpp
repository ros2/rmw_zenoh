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

#include <memory>
#include <utility>
#include <vector>

#include "rmw_data_types.hpp"

std::mutex sub_callback_mutex;

///==============================================================================
void sub_data_handler(
  const z_sample_t * sample,
  void * data)
{
  z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
  // TODO(yadunund): Remove after debugging.
  printf(
    ">> [Subscriber] Received ('%s': size: '%i', payload: '%s')\n", z_loan(keystr),
    static_cast<int>(sample->payload.len), sample->payload.start);

  auto sub_data = static_cast<rmw_subscription_data_t *>(data);
  if (sub_data == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> guard(sub_callback_mutex);
  // Vector to store the byte array (so we have a copyable container instead of a pointer)
  std::vector<unsigned char> byte_vec(sample->payload.start,
    sample->payload.start + sample->payload.len);
  auto byte_vec_ptr = std::make_shared<std::vector<unsigned char>>(std::move(byte_vec));

  std::unique_lock<std::mutex> lock(sub_data->message_queue_mutex);

  if (sub_data->message_queue.size() >= sub_data->queue_depth) {
    // Log warning if message is discarded due to hitting the queue depth
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "Message queue depth of %ld reached, discarding oldest message "
      "for subscription for %s",
      sub_data->queue_depth,
      z_loan(keystr));

    sub_data->message_queue.pop_back();
  }
  sub_data->message_queue.push_front(byte_vec_ptr);

  sub_data->message_queue_mutex.unlock();

  z_drop(z_move(keystr));
}
