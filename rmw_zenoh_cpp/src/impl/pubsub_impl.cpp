#include "pubsub_impl.hpp"

#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rmw_zenoh_cpp/TypeSupport.hpp"
#include "rcutils/logging_macros.h"

extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

std::mutex sub_callback_mutex;

size_t rmw_subscription_data_t::sub_id_counter = 0;

// Zenoh topic to subscription data
std::unordered_map<std::string, std::vector<rmw_subscription_data_t *> >
  rmw_subscription_data_t::zn_topic_to_sub_data_map;

// Static message map
std::unordered_map<std::string, std::vector<unsigned char> >
  rmw_subscription_data_t::zn_messages_;

void rmw_subscription_data_t::zn_sub_callback(const zn_sample * sample) {
  // Prevent race conditions...
  std::lock_guard<std::mutex> guard(sub_callback_mutex);

  // NOTE(CH3): We unfortunately have to do this copy construction since we shouldn't be using
  // char * as keys to the unordered_map
  std::string key(sample->key.val, sample->key.len);

  // Vector to store the byte array (so we have a copyable type instead of a pointer)
  std::vector<unsigned char> byte_vec(sample->value.val, sample->value.val + sample->value.len);

  // Fill the static message map with the latest received message
  //
  // NOTE(CH3): This means that the queue size for each topic is ONE for now!!
  // So this might break if a topic is being spammed.
  // TODO(CH3): Implement queuing logic
  if (rmw_subscription_data_t::zn_messages_.find(key)
      != rmw_subscription_data_t::zn_messages_.end()) {
    // Log warning if message is clobbered
    RCUTILS_LOG_WARN_NAMED(
        "rmw_zenoh_cpp", "overwriting existing untaken zenoh message: %s", key.c_str());
  }

  rmw_subscription_data_t::zn_messages_[key] = std::vector<unsigned char>(byte_vec);
}
