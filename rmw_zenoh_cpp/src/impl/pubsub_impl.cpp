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

#include "pubsub_impl.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rmw_zenoh_cpp/TypeSupport.hpp"
#include "rcutils/logging_macros.h"

std::mutex sub_callback_mutex;


/// STATIC SUBSCRIPTION DATA MEMBERS ===========================================
std::atomic<size_t> rmw_subscription_data_t::subscription_id_counter(0);

// *INDENT-OFF* because uncrustify can't decide which way to format this
// Map of Zenoh topic key expression to subscription data
std::unordered_map<std::string, std::vector<rmw_subscription_data_t *>>
  rmw_subscription_data_t::zn_topic_to_sub_data;
// *INDENT-ON*


/// ZENOH MESSAGE SUBSCRIPTION CALLBACK (static method) ========================
void rmw_subscription_data_t::zn_sub_callback(const zn_sample * sample)
{
  std::lock_guard<std::mutex> guard(sub_callback_mutex);

  // NOTE(CH3): We unfortunately have to do this copy construction since we shouldn't be using
  // char * as keys to the unordered_map
  std::string key(sample->key.val, sample->key.len);

  // Vector to store the byte array (so we have a copyable container instead of a pointer)
  std::vector<unsigned char> byte_vec(sample->value.val, sample->value.val + sample->value.len);

  // Get shared pointer to byte array vector
  // NOTE(CH3): We use a shared pointer to avoid copies and to leverage on the smart pointer's
  // reference counting
  auto byte_vec_ptr = std::make_shared<std::vector<unsigned char>>(std::move(byte_vec));

  auto map_iter = rmw_subscription_data_t::zn_topic_to_sub_data.find(key);

  // If the key was not found in the map, it means that there are no RMW subscriptions listening
  // on this topic, so this message can be dropped without issue
  if (map_iter != rmw_subscription_data_t::zn_topic_to_sub_data.end()) {
    // Push shared pointer to message bytes to all associated subscription message queues
    for (auto it = map_iter->second.begin(); it != map_iter->second.end(); ++it) {
      std::unique_lock<std::mutex> lock((*it)->message_queue_mutex_);

      if ((*it)->zn_message_queue_.size() >= (*it)->queue_depth_) {
        // Log warning if message is discarded due to hitting the queue depth
        RCUTILS_LOG_WARN_NAMED(
          "rmw_zenoh_cpp",
          "Message queue depth of %ld reached, discarding oldest message "
          "for subscription for %s (ID: %ld)",
          (*it)->queue_depth_,
          key.c_str(),
          (*it)->subscription_id_);

        (*it)->zn_message_queue_.pop_back();
      }
      (*it)->zn_message_queue_.push_front(byte_vec_ptr);

      (*it)->message_queue_mutex_.unlock();
    }
  }
}
