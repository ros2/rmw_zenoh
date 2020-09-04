#include "client_impl.hpp"

#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rcutils/logging_macros.h"

std::mutex response_callback_mutex;
std::mutex query_callback_mutex;

// Static sequence ID
std::atomic<std::int64_t> rmw_client_data_t::sequence_id_counter(0);

// Static response ROS message map
std::unordered_map<std::string, std::vector<unsigned char>>
  rmw_client_data_t::zn_response_messages_;

// Static availability query Zenoh response set
std::unordered_set<std::string> rmw_client_data_t::zn_availability_query_responses_;

/// ZENOH RESPONSE SUBSCRIPTION CALLBACK =======================================
void rmw_client_data_t::zn_response_sub_callback(const zn_sample * sample) {
  // Prevent race conditions...
  std::lock_guard<std::mutex> guard(response_callback_mutex);

  // NOTE(CH3): We unfortunately have to do this copy construction since we shouldn't be using
  // char * as keys to the unordered_map
  std::string key(sample->key.val, sample->key.len);

  // Vector to store the byte array (so we have a copyable type instead of a pointer)
  std::vector<unsigned char> byte_vec(sample->value.val, sample->value.val + sample->value.len);

  // Fill the static response message map with the latest received message
  //
  // NOTE(CH3): This means that the queue size for each topic is ONE for now!!
  // So this might break if a service is being spammed.
  // TODO(CH3): Implement queuing logic
  if (rmw_client_data_t::zn_response_messages_.find(key)
      != rmw_client_data_t::zn_response_messages_.end()) {
    // Log warning if message is clobbered
    RCUTILS_LOG_WARN_NAMED(
        "rmw_zenoh_cpp", "overwriting existing untaken zenoh response message: %s", key.c_str());
  }

  rmw_client_data_t::zn_response_messages_[key] = std::vector<unsigned char>(byte_vec);
}

/// ZENOH SERVICE AVAILABILITY QUERY CALLBACK ==================================
void rmw_client_data_t::zn_service_availability_query_callback(const zn_source_info * info,
                                                               const zn_sample * sample) {
  // Prevent race conditions...
  std::lock_guard<std::mutex> guard(query_callback_mutex);

  // NOTE(CH3): We unfortunately have to do this copy construction since we shouldn't be using
  // char * as keys to the unordered_map
  std::string key(sample->key.val, sample->key.len);

  // Insert if key not found in query response set
  if (rmw_client_data_t::zn_availability_query_responses_.find(key)
      == rmw_client_data_t::zn_availability_query_responses_.end()) {
    rmw_client_data_t::zn_availability_query_responses_.insert(key);
  } else {
    RCUTILS_LOG_WARN_NAMED("rmw_zenoh_cpp", "zenoh availability for %s already set", key.c_str());
  }
}
