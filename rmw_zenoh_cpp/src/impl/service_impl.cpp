#include "service_impl.hpp"

#include <iostream>

#include "rcutils/logging_macros.h"
#include "rmw_zenoh_cpp/TypeSupport.hpp"

extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

std::mutex request_callback_mutex;

/// STATIC SERVICE DATA MEMBERS ================================================
std::atomic<size_t> rmw_service_data_t::service_id_counter(0);

// Map of Zenoh topic key expression to service data
std::unordered_map<std::string, std::vector<rmw_service_data_t *>>
  rmw_service_data_t::zn_topic_to_service_data;


/// ZENOH REQUEST MESSAGE SUBSCRIPTION CALLBACK (static method) ================
void rmw_service_data_t::zn_request_sub_callback(const zn_sample * sample) {
  std::lock_guard<std::mutex> guard(request_callback_mutex);

  // NOTE(CH3): We unfortunately have to do this copy construction since we shouldn't be using
  // char * as keys to the unordered_map
  std::string key(sample->key.val, sample->key.len);

  // Vector to store the byte array (so we have a copyable container instead of a pointer)
  std::vector<unsigned char> byte_vec(sample->value.val, sample->value.val + sample->value.len);

  // Get shared pointer to byte array vector
  // NOTE(CH3): We use a shared pointer to avoid copies and to leverage on the smart pointer's
  // reference counting
  auto byte_vec_ptr = std::make_shared<std::vector<unsigned char>>(std::move(byte_vec));

  auto map_iter = rmw_service_data_t::zn_topic_to_service_data.find(key);

  // If the key was not found in the map, the map_iter will return a pointer to the map's end
  if (map_iter != rmw_service_data_t::zn_topic_to_service_data.end()) {
    // Push shared pointer to message bytes to all associated service request message queues
    for (auto it = map_iter->second.begin(); it != map_iter->second.end(); ++it) {
      std::unique_lock<std::mutex> lock((*it)->request_queue_mutex_);

      if ((*it)->zn_request_message_queue_.size() >= (*it)->queue_depth_) {
        // Log warning if message is discarded due to hitting the queue depth
        RCUTILS_LOG_WARN_NAMED(
          "rmw_zenoh_cpp",
          "Request queue depth of %ld reached, discarding oldest request message "
          "for service for %s (ID: %ld)",
          (*it)->queue_depth_,
          key.c_str(),
          (*it)->service_id_);

        (*it)->zn_request_message_queue_.pop_back();
      }
      (*it)->zn_request_message_queue_.push_front(byte_vec_ptr);

      (*it)->request_queue_mutex_.unlock();
    }
  }
}

/// ZENOH SERVICE AVAILABILITY QUERYABLE CALLBACK ==============================
void rmw_service_data_t::zn_service_availability_queryable_callback(ZNQuery * query) {
  const zn_string * resource = zn_query_res_name(query);
  const zn_string * predicate = zn_query_predicate(query);

  std::string res(resource->val, resource->len);
  std::string response("available");  // NOTE(CH3): The contents actually don't matter...

  zn_send_reply(query, res.c_str(), (const unsigned char *)response.c_str(), response.length());
}
