// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef DETAIL__RMW_DATA_TYPES_HPP_
#define DETAIL__RMW_DATA_TYPES_HPP_

#include <zenoh.h>

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>

#include "rcutils/allocator.h"

#include "rmw/rmw.h"

#include "graph_cache.hpp"
#include "message_type_support.hpp"

/// Structs for various type erased data fields.

///==============================================================================
struct rmw_context_impl_s
{
  // An owned session.
  z_owned_session_t session;

  // The SHM manager.
  zc_owned_shm_manager_t shm_manager;

  z_owned_subscriber_t graph_subscriber;

  /// Shutdown flag.
  bool is_shutdown;

  // Equivalent to rmw_dds_common::Context's guard condition
  /// Guard condition that should be triggered when the graph changes.
  rmw_guard_condition_t * graph_guard_condition;

  GraphCache graph_cache;
};

///==============================================================================
struct rmw_node_data_t
{
  // TODO(yadunund): Add a GraphCache object.

  // Map topic name to topic types.
  std::unordered_set<std::unordered_set<std::string>> publishers;
  std::unordered_set<std::unordered_set<std::string>> subscriptions;
};

///==============================================================================
struct rmw_publisher_data_t
{
  // An owned publisher.
  z_owned_publisher_t pub;

  // Type support fields
  const void * type_support_impl;
  const char * typesupport_identifier;
  MessageTypeSupport * type_support;

  // Context for memory allocation for messages.
  rmw_context_t * context;

  uint64_t graph_cache_handle;
};

///==============================================================================
struct rmw_wait_set_data_t
{
  std::condition_variable condition_variable;
  std::mutex condition_mutex;

  rmw_context_t * context;
};

///==============================================================================
// z_owned_closure_sample_t
void sub_data_handler(const z_sample_t * sample, void * sub_data);

struct saved_msg_data
{
  explicit saved_msg_data(zc_owned_payload_t p, uint64_t recv_ts, const uint8_t pub_gid[16])
  : payload(p), recv_timestamp(recv_ts)
  {
    memcpy(publisher_gid, pub_gid, 16);
  }

  zc_owned_payload_t payload;
  uint64_t recv_timestamp;
  uint8_t publisher_gid[16];
};

///==============================================================================
struct rmw_subscription_data_t
{
  z_owned_subscriber_t sub;

  const void * type_support_impl;
  const char * typesupport_identifier;
  MessageTypeSupport * type_support;
  rmw_context_t * context;

  std::deque<std::unique_ptr<saved_msg_data>> message_queue;
  std::mutex message_queue_mutex;

  size_t queue_depth;
  bool reliable;

  std::mutex internal_mutex;
  std::condition_variable * condition{nullptr};

  uint64_t graph_cache_handle;
};

#endif  // DETAIL__RMW_DATA_TYPES_HPP_
