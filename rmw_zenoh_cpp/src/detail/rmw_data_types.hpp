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
#include <vector>

#include "rmw/rmw.h"

#include "TypeSupport.hpp"

/// Structs for various type erased data fields.

///==============================================================================
struct rmw_context_impl_s
{
  // An owned session.
  z_owned_session_t session;

  /// Shutdown flag.
  bool is_shutdown;

  // Equivalent to rmw_dds_common::Context's guard condition
  /// Guard condition that should be triggered when the graph changes.
  rmw_guard_condition_t * graph_guard_condition;
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
  TypeSupport * type_support;

  // Context for memory allocation for messages.
  rmw_context_t * context;
};

///==============================================================================
struct rmw_wait_set_data_t
{
  std::condition_variable condition;
  std::mutex condition_mutex;
};

///==============================================================================
// z_owned_closure_sample_t
void sub_data_handler(const z_sample_t * sample, void * sub_data);

///==============================================================================
struct rmw_subscription_data_t
{
  z_owned_subscriber_t sub;

  const void * type_support_impl;
  const char * typesupport_identifier;
  TypeSupport * type_support;

  std::deque<std::shared_ptr<std::vector<unsigned char>>> message_queue;
  std::mutex message_queue_mutex;

  size_t queue_depth;
  bool reliable;
};

#endif  // DETAIL__RMW_DATA_TYPES_HPP_
