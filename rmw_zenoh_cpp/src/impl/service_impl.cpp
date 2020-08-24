extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

#include <iostream>
#include <mutex>

#include "rmw_zenoh_cpp/TypeSupport.hpp"
#include "service_impl.hpp"

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
  rmw_service_data_t::zn_request_messages_[key] = std::vector<unsigned char>(byte_vec);
}
