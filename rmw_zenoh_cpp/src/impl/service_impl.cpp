#include "service_impl.hpp"

#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rcutils/logging_macros.h"
#include "rmw_zenoh_cpp/TypeSupport.hpp"

extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

std::mutex request_callback_mutex;

// Static request message map
std::unordered_map<std::string, std::vector<unsigned char> >
  rmw_service_data_t::zn_request_messages_;

void rmw_service_data_t::zn_request_sub_callback(const zn_sample * sample) {
  // Prevent race conditions...
  std::lock_guard<std::mutex> guard(request_callback_mutex);

  // NOTE(CH3): We unfortunately have to do this copy construction since we shouldn't be using
  // char * as keys to the unordered_map
  std::string key(sample->key.val, sample->key.len);

  // Vector to store the byte array (so we have a copyable type instead of a pointer)
  std::vector<unsigned char> byte_vec(sample->value.val, sample->value.val + sample->value.len);

  // Fill the static request message map with the latest received message
  //
  // NOTE(CH3): This means that the queue size for each topic is ONE for now!!
  // So this might break if a service is being spammed.
  // TODO(CH3): Implement queuing logic
  if (rmw_service_data_t::zn_request_messages_.find(key)
      != rmw_service_data_t::zn_request_messages_.end()) {
    // Log warning if message is clobbered
    RCUTILS_LOG_WARN_NAMED(
        "rmw_zenoh_cpp", "overwriting existing untaken zenoh request message: %s", key.c_str());
  }

  rmw_service_data_t::zn_request_messages_[key] = std::vector<unsigned char>(byte_vec);
}
