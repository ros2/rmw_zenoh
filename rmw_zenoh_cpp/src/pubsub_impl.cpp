extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

#include "rmw_zenoh_cpp/TypeSupport.hpp"
#include "pubsub_impl.hpp"
#include <iostream>

// Declaring static members
void rmw_subscription_data_t::zn_sub_callback(const zn_sample * sample) {
  // NOTE(CH3): We might need a mutex here to prevent race conditions...

  // NOTE(CH3): We unfortunately have to do this copy construction since we shouldn't be using
  // char * as keys to the unordered_map
  std::string key(sample->key.val, sample->key.len);

  // Vector to store the byte array (so we have a copyable type instead of a pointer)
  std::vector<unsigned char> byte_vec(sample->value.val, sample->value.val + sample->value.len);

  rmw_subscription_data_t::zn_messages_[key] = std::pair<std::vector<unsigned char>, size_t>(
    byte_vec,
    sample->value.len
  );
}

std::unordered_map<std::string, std::pair<std::vector<unsigned char>, size_t> >
  rmw_subscription_data_t::zn_messages_;
