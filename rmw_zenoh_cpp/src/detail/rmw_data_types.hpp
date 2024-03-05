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
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "rcutils/allocator.h"

#include "rmw/rmw.h"

#include "graph_cache.hpp"
#include "message_type_support.hpp"
#include "service_type_support.hpp"

/// Structs for various type erased data fields.

///==============================================================================
struct rmw_context_impl_s
{
  // An owned session.
  z_owned_session_t session;

  // An optional SHM manager that is initialized of SHM is enabled in the
  // zenoh session config.
  std::optional<zc_owned_shm_manager_t> shm_manager;

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
  // TODO(Yadunund): Do we need a token at the node level? Right now I have one
  // for cases where a node may spin up but does not have any publishers or subscriptions.
  // Liveliness token for the node.
  zc_owned_liveliness_token_t token;
};

///==============================================================================
class rmw_publisher_data_t final
{
public:
  // An owned publisher.
  z_owned_publisher_t pub;

  // Optional publication cache when durability is transient_local.
  std::optional<ze_owned_publication_cache_t> pub_cache;

  // Store the actual QoS profile used to configure this publisher.
  rmw_qos_profile_t adapted_qos_profile;

  // Liveliness token for the publisher.
  zc_owned_liveliness_token_t token;

  // Type support fields
  const void * type_support_impl;
  const char * typesupport_identifier;
  MessageTypeSupport * type_support;

  // Context for memory allocation for messages.
  rmw_context_t * context;

  uint8_t pub_gid[RMW_GID_STORAGE_SIZE];

  size_t get_next_sequence_number();

private:
  std::mutex sequence_number_mutex_;
  size_t sequence_number_{1};
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
  explicit saved_msg_data(
    zc_owned_payload_t p,
    uint64_t recv_ts,
    const uint8_t pub_gid[RMW_GID_STORAGE_SIZE],
    int64_t seqnum,
    int64_t source_ts);

  ~saved_msg_data();

  zc_owned_payload_t payload;
  uint64_t recv_timestamp;
  uint8_t publisher_gid[RMW_GID_STORAGE_SIZE];
  int64_t sequence_number;
  int64_t source_timestamp;
};

///==============================================================================
class rmw_subscription_data_t final
{
public:
  // An owned subscriber or querying_subscriber depending on the QoS settings.
  std::variant<z_owned_subscriber_t, ze_owned_querying_subscriber_t> sub;

  // Store the actual QoS profile used to configure this subscription.
  rmw_qos_profile_t adapted_qos_profile;

  // Liveliness token for the subscription.
  zc_owned_liveliness_token_t token;

  const void * type_support_impl;
  const char * typesupport_identifier;
  MessageTypeSupport * type_support;
  rmw_context_t * context;

  void attach_condition(std::condition_variable * condition_variable);

  void detach_condition();

  bool message_queue_is_empty() const;

  std::unique_ptr<saved_msg_data> pop_next_message();

  void add_new_message(std::unique_ptr<saved_msg_data> msg, const std::string & topic_name);

private:
  std::deque<std::unique_ptr<saved_msg_data>> message_queue_;
  mutable std::mutex message_queue_mutex_;

  void notify();

  std::condition_variable * condition_{nullptr};
  std::mutex condition_mutex_;
};


///==============================================================================

void service_data_handler(const z_query_t * query, void * service_data);

void client_data_handler(z_owned_reply_t * reply, void * client_data);

///==============================================================================

class ZenohQuery final
{
public:
  ZenohQuery(const z_query_t * query);

  ~ZenohQuery();

  const z_query_t get_query() const;

private:
  z_owned_query_t query_;
};

class rmw_service_data_t final
{
public:
  z_owned_keyexpr_t keyexpr;
  z_owned_queryable_t qable;

  // Store the actual QoS profile used to configure this service.
  // The QoS is reused for getting requests and sending responses.
  rmw_qos_profile_t adapted_qos_profile;

  // Liveliness token for the service.
  zc_owned_liveliness_token_t token;

  const void * request_type_support_impl;
  const void * response_type_support_impl;
  const char * typesupport_identifier;
  RequestTypeSupport * request_type_support;
  ResponseTypeSupport * response_type_support;

  rmw_context_t * context;

  bool query_queue_is_empty() const;

  void attach_condition(std::condition_variable * condition_variable);

  void detach_condition();

  std::unique_ptr<ZenohQuery> pop_next_query();

  void add_new_query(std::unique_ptr<ZenohQuery> query);

  bool add_to_query_map(int64_t sequence_number, std::unique_ptr<ZenohQuery> query);

  std::unique_ptr<ZenohQuery> take_from_query_map(int64_t sequence_number);

private:
  void notify();

  // Deque to store the queries in the order they arrive.
  std::deque<std::unique_ptr<ZenohQuery>> query_queue_;
  mutable std::mutex query_queue_mutex_;

  // Map to store the sequence_number -> query_id
  std::unordered_map<int64_t, std::unique_ptr<ZenohQuery>> sequence_to_query_map_;
  std::mutex sequence_to_query_map_mutex_;

  std::condition_variable * condition_{nullptr};
  std::mutex condition_mutex_;
};

///==============================================================================

class ZenohReply final
{
public:
  ZenohReply(const z_owned_reply_t * reply);

  ~ZenohReply();

  std::optional<z_sample_t> get_sample() const;

private:
  z_owned_reply_t reply_;
};

class rmw_client_data_t final
{
public:
  z_owned_keyexpr_t keyexpr;
  z_owned_closure_reply_t zn_closure_reply;

  // Store the actual QoS profile used to configure this client.
  // The QoS is reused for sending requests and getting responses.
  rmw_qos_profile_t adapted_qos_profile;

  // Liveliness token for the client.
  zc_owned_liveliness_token_t token;

  const void * request_type_support_impl;
  const void * response_type_support_impl;
  const char * typesupport_identifier;
  RequestTypeSupport * request_type_support;
  ResponseTypeSupport * response_type_support;

  rmw_context_t * context;

  uint8_t client_gid[RMW_GID_STORAGE_SIZE];

  size_t get_next_sequence_number();

  void add_new_reply(std::unique_ptr<ZenohReply> reply);

  bool reply_queue_is_empty() const;

  void attach_condition(std::condition_variable * condition_variable);

  void detach_condition();

  std::unique_ptr<ZenohReply> pop_next_reply();

private:
  void notify();

  size_t sequence_number_{1};
  std::mutex sequence_number_mutex_;

  std::condition_variable * condition_{nullptr};
  std::mutex condition_mutex_;

  std::deque<std::unique_ptr<ZenohReply>> reply_queue_;
  mutable std::mutex reply_queue_mutex_;
};

#endif  // DETAIL__RMW_DATA_TYPES_HPP_
