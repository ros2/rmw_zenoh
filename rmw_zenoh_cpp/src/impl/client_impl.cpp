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

#include "client_impl.hpp"

#include <iostream>

#include "rcutils/logging_macros.h"

std::mutex response_callback_mutex;
std::mutex query_callback_mutex;

/// STATIC CLIENT DATA MEMBERS ================================================
std::atomic<std::int64_t> rmw_client_data_t::sequence_id_counter(0);
std::atomic<size_t> rmw_client_data_t::client_id_counter(0);

// *INDENT-OFF* because uncrustify can't decide which way to format this
// Map of Zenoh topic key expression to client data
std::unordered_map<std::string, std::vector<rmw_client_data_t *>>
  rmw_client_data_t::zn_topic_to_client_data;
// *INDENT-ON*

// *INDENT-OFF* because uncrustify can't decide which way to format this
// Map of Zenoh queryable key expression to client data
std::unordered_map<std::string, std::vector<rmw_client_data_t *>>
  rmw_client_data_t::zn_queryable_to_client_data;
// *INDENT-ON*

/// ZENOH RESPONSE SUBSCRIPTION CALLBACK (static method) =======================
void rmw_client_data_t::zn_response_sub_callback(const zn_sample * sample)
{
  std::lock_guard<std::mutex> guard(response_callback_mutex);

  // NOTE(CH3): We unfortunately have to do this copy construction since we shouldn't be using
  // char * as keys to the unordered_map
  std::string key(sample->key.val, sample->key.len);

  // Vector to store the byte array (so we have a copyable container instead of a pointer)
  std::vector<unsigned char> byte_vec(sample->value.val, sample->value.val + sample->value.len);

  // Get shared pointer to byte array vector
  // NOTE(CH3): We use a shared pointer to avoid copies and to leverage on the smart pointer's
  // reference counting
  auto byte_vec_ptr = std::make_shared<std::vector<unsigned char>>(std::move(byte_vec));

  auto map_iter = rmw_client_data_t::zn_topic_to_client_data.find(key);

  // If the key was not found in the map, it means that there are no RMW clients listening on this
  // topic, so this message can be dropped without issue
  if (map_iter != rmw_client_data_t::zn_topic_to_client_data.end()) {
    // Push shared pointer to message bytes to all associated client response message queues
    for (auto it = map_iter->second.begin(); it != map_iter->second.end(); ++it) {
      std::unique_lock<std::mutex> lock((*it)->response_queue_mutex_);

      if ((*it)->zn_response_message_queue_.size() >= (*it)->queue_depth_) {
        // Log warning if message is discarded due to hitting the queue depth
        RCUTILS_LOG_WARN_NAMED(
          "rmw_zenoh_cpp",
          "Request queue depth of %ld reached, discarding oldest response message "
          "for client for %s (ID: %ld)",
          (*it)->queue_depth_,
          key.c_str(),
          (*it)->client_id_);

        (*it)->zn_response_message_queue_.pop_back();
      }
      (*it)->zn_response_message_queue_.push_front(byte_vec_ptr);

      (*it)->response_queue_mutex_.unlock();
    }
  }
}
/// ZENOH SERVICE AVAILABILITY QUERY CALLBACK ==================================
void rmw_client_data_t::zn_service_availability_query_callback(
  const zn_source_info * info,
  const zn_sample * sample)
{
  std::lock_guard<std::mutex> guard(query_callback_mutex);

  // NOTE(CH3): We unfortunately have to do this copy construction since we shouldn't be using
  // char * as keys to the unordered_map
  std::string key(sample->key.val, sample->key.len);

  auto map_iter = rmw_client_data_t::zn_queryable_to_client_data.find(key);

  if (map_iter != rmw_client_data_t::zn_queryable_to_client_data.end()) {
    // Update all associated client availability query response sets
    for (auto it = map_iter->second.begin(); it != map_iter->second.end(); ++it) {
      std::unique_lock<std::mutex> lock((*it)->availability_set_mutex_);

      (*it)->zn_availability_query_responses_.insert(key);

      (*it)->availability_set_mutex_.unlock();
    }
  }
}
