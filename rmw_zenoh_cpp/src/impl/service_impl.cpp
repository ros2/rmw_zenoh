extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

#include "rmw_zenoh_cpp/TypeSupport.hpp"
#include "service_impl.hpp"
#include <iostream>
#include <mutex>

std::mutex request_callback_mutex;

// Declaring static members
void rmw_service_data_t::zn_request_sub_callback(const zn_sample * sample) {
  // Prevent race conditions...
  std::lock_guard<std::mutex> guard(request_callback_mutex);

  // NOTE(CH3): We unfortunately have to do this copy construction since we shouldn't be using
  // char * as keys to the unordered_map
  std::string key(sample->key.val, sample->key.len);

  // Vector to store the byte array (so we have a copyable type instead of a pointer)
  std::vector<unsigned char> byte_vec(sample->value.val, sample->value.val + sample->value.len);

  rmw_service_data_t::zn_request_messages_[key] = std::vector<unsigned char>(byte_vec);
}

std::unordered_map<std::string, std::vector<unsigned char> >
  rmw_service_data_t::zn_request_messages_;
