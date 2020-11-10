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

#ifndef IMPL__SERVICE_IMPL_HPP_
#define IMPL__SERVICE_IMPL_HPP_

#include <unordered_map>
#include <utility>
#include <string>
#include <vector>
#include <memory>
#include <deque>
#include <mutex>
#include <atomic>

#include "rmw/rmw.h"
#include "rmw_zenoh_cpp/TypeSupport.hpp"

extern "C"
{
#ifdef USE_ZENOH_PICO
#include "zenoh/net/private/unix/types.h"
#include "zenoh.h"
#else
#include "zenoh/zenoh.h"
#endif
}

struct rmw_service_data_t
{
  /// STATIC MEMBERS ===========================================================
  static void zn_request_sub_callback(const zn_sample_t * sample, const void * arg);

  static void zn_service_availability_queryable_callback(zn_query_t * query, const void * arg);

  // Counter to give service servers unique IDs
  static std::atomic<size_t> service_id_counter;

  // *INDENT-OFF* because uncrustify can't decide which way to format this
  // Map of Zenoh topic key expression to service data struct instances
  static std::unordered_map<std::string, std::vector<rmw_service_data_t *>>
    zn_topic_to_service_data;
  // *INDENT-ON*

  /// INSTANCE MEMBERS =========================================================
  // Type support
  const void * request_type_support_impl_;
  const void * response_type_support_impl_;
  const char * typesupport_identifier_;

  rmw_zenoh_cpp::TypeSupport * request_type_support_;
  rmw_zenoh_cpp::TypeSupport * response_type_support_;

  /// ZENOH ====================================================================
  zn_session_t * zn_session_;
  zn_queryable_t * zn_queryable_;

  // Request Sub
  const char * zn_request_topic_key_;
  zn_subscriber_t * zn_request_subscriber_;

  // Response Pub
  const char * zn_response_topic_key_;
  size_t zn_response_topic_id_;

  /// ROS ======================================================================
  const rmw_node_t * node_;

  // Instanced request message queue
  std::deque<std::shared_ptr<std::vector<unsigned char>>> zn_request_message_queue_;
  std::mutex request_queue_mutex_;

  size_t service_id_;
  size_t queue_depth_;
};

#endif  // IMPL__SERVICE_IMPL_HPP_
