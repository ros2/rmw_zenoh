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

#ifndef IMPL__CLIENT_IMPL_HPP_
#define IMPL__CLIENT_IMPL_HPP_

#include <unordered_map>
#include <unordered_set>
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
  #include "zenoh/zenoh-ffi.h"
}

struct rmw_client_data_t
{
  /// STATIC MEMBERS ===========================================================
  static void zn_response_sub_callback(const zn_sample * sample);
  static void zn_service_availability_query_callback(
    const zn_source_info * info, const zn_sample * sample
  );

  // Counter to give client servers unique IDs
  static std::atomic<size_t> client_id_counter;

  // Request-response sequence id (To identify and match individual requests)
  static std::atomic<std::int64_t> sequence_id_counter;

  // Map of Zenoh topic key expression to client data struct instances
  static std::unordered_map<std::string, std::vector<rmw_client_data_t *>>
    zn_topic_to_client_data;

  // Map of Zenoh queryable key expression to client data struct instances
  static std::unordered_map<std::string, std::vector<rmw_client_data_t *>>
    zn_queryable_to_client_data;

  /// TYPE SUPPORT =============================================================
  const void * request_type_support_impl_;
  const void * response_type_support_impl_;
  const char * typesupport_identifier_;

  rmw_zenoh_cpp::TypeSupport * request_type_support_;
  rmw_zenoh_cpp::TypeSupport * response_type_support_;

  /// ZENOH ====================================================================
  ZNSession * zn_session_;

  // Response Sub
  const char * zn_response_topic_key_;
  ZNSubscriber * zn_response_subscriber_;

  // Request Pub
  const char * zn_request_topic_key_;
  size_t zn_request_topic_id_;

  /// ROS ======================================================================
  const rmw_node_t * node_;

  // Instanced response message queue
  std::deque<std::shared_ptr<std::vector<unsigned char>>> zn_response_message_queue_;
  std::mutex response_queue_mutex_;

  // Instanced availability query Zenoh responses
  std::unordered_set<std::string> zn_availability_query_responses_;
  std::mutex availability_set_mutex_;

  size_t client_id_;
  size_t queue_depth_;
};

#endif  // IMPL__CLIENT_IMPL_HPP_
