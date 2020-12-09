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

#ifndef IMPL__PUBSUB_IMPL_HPP_
#define IMPL__PUBSUB_IMPL_HPP_

#include <unordered_map>
#include <utility>
#include <string>
#include <vector>
#include <memory>
#include <deque>
#include <mutex>
#include <atomic>

#include "rmw/rmw.h"
#include "rmw_zenoh_common_cpp/TypeSupport.hpp"

extern "C"
{
#include "rmw_zenoh_common_cpp/zenoh-net-interface.h"
}

struct rmw_node_impl_t
{
};

struct rmw_publisher_data_t
{
  const void * type_support_impl_;
  const char * typesupport_identifier_;

  rmw_zenoh_common_cpp::TypeSupport * type_support_;

  size_t zn_topic_id_;
  zn_session_t * zn_session_;

  const rmw_node_t * node_;
};

// Functionally a struct. But with a method for handling incoming Zenoh messages
struct rmw_subscription_data_t
{
  /// STATIC MEMBERS ===============================================================================
  static void zn_sub_callback(const zn_sample_t * sample, const void * arg);

  // Counter to give subscriptions unique IDs
  static std::atomic<size_t> subscription_id_counter;

  // *INDENT-OFF* because uncrustify can't decide which way to format this
  // Map of Zenoh topic key expression to subscription data struct instances
  static std::unordered_map<std::string, std::vector<rmw_subscription_data_t *>>
    zn_topic_to_sub_data;
  // *INDENT-ON*

  /// INSTANCE MEMBERS =============================================================================
  const void * type_support_impl_;
  const char * typesupport_identifier_;

  rmw_zenoh_common_cpp::TypeSupport * type_support_;
  const rmw_node_t * node_;

  zn_session_t * zn_session_;
  zn_subscriber_t * zn_subscriber_;

  // Instanced message queue
  std::deque<std::shared_ptr<std::vector<unsigned char>>> zn_message_queue_;
  std::mutex message_queue_mutex_;

  size_t subscription_id_;
  size_t queue_depth_;
};

#endif  // IMPL__PUBSUB_IMPL_HPP_
