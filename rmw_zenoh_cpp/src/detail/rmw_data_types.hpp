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

#include "rmw/event_callback_type.h"
#include "rmw/rmw.h"

#include "graph_cache.hpp"
#include "message_type_support.hpp"
#include "service_type_support.hpp"

/// Structs for various type erased data fields.

/// A struct to store status changes which can be mapped to rmw event statuses.
/// The data field can be used to store serialized information for more complex statuses.
struct rmw_zenoh_event_status_t
{
  size_t total_count;
  size_t total_count_change;
  size_t current_count;
  std::string data;
};

///=============================================================================
/// Base class to be inherited by entities that support events.
class EventsBase
{
public:
  /// @brief Set the user defined callback that should be called when
  /// a new message/response/request is received.
  /// @param user_data the data that should be passed to the callback.
  /// @param callback the callback to be set.
  void set_user_callback(const void * user_data, rmw_event_callback_t callback);

  /// Trigger the user callback.
  void trigger_user_callback();

  /// @brief  Set the callback to be triggered when the relevant event is triggered.
  /// @param event_id the id of the event
  /// @param callback the callback to trigger for this event.
  /// @param user_data the data to be passed to the callback.
  void event_set_callback(
    rmw_zenoh_event_type_t event_id,
    rmw_event_callback_t callback,
    const void * user_data);

  /// @brief Trigger the callback for an event.
  /// @param event_id the event id whose callback should be triggered.
  void trigger_event_callback(rmw_zenoh_event_type_t event_id);

  /// @brief  Returns true if the event queue is empty.
  /// @param event_id the event id whose event queue should be checked.
  bool event_queue_is_empty(rmw_zenoh_event_type_t event_id) const;

  /// Pop the next event in the queue.
  /// @param event_id the event id whose queue should be popped.
  std::unique_ptr<rmw_zenoh_event_status_t> pop_next_event(
    rmw_zenoh_event_type_t event_id);

  /// Add an event status for an event.
  /// @param event_id the event id queue to which the status should be added.
  void add_new_event(
    rmw_zenoh_event_type_t event_id,
    std::unique_ptr<rmw_zenoh_event_status_t> event);

  /// @brief Attach the condition variable provided by rmw_wait.
  /// @param condition_variable to attach.
  void attach_event_condition(
    rmw_zenoh_event_type_t event_id,
    std::condition_variable * condition_variable);

  /// @brief Detach the condition variable provided by rmw_wait.
  void detach_event_condition(rmw_zenoh_event_type_t event_id);

private:
  /// Notify once event is added to an event queue.
  void notify_event(rmw_zenoh_event_type_t event_id);

  /// Mutex to lock when read/writing members.
  mutable std::mutex event_mutex_;
  /// Mutex to lock for event_condition.
  mutable std::mutex event_condition_mutex_;
  /// Condition variable to attach for event notifications.
  std::condition_variable * event_conditions_[ZENOH_EVENT_ID_MAX + 1]{nullptr};
  /// User callback that can be set via set_user_callback().
  rmw_event_callback_t callback_ {nullptr};
  /// User data that should be passed to the user callback.
  const void * user_data_ {nullptr};
  /// Count for
  size_t unread_count_ {0};
  rmw_event_callback_t event_callback_[ZENOH_EVENT_ID_MAX + 1] {nullptr};
  const void * event_data_[ZENOH_EVENT_ID_MAX + 1] {nullptr};
  size_t event_unread_count_[ZENOH_EVENT_ID_MAX + 1] {0};
  // A dequeue of events for each type of event this RMW supports.
  std::deque<std::unique_ptr<rmw_zenoh_event_status_t>> event_queues_[ZENOH_EVENT_ID_MAX + 1] {};
  const std::size_t event_queue_depth_ = 10;
};

///=============================================================================
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

///=============================================================================
struct rmw_node_data_t
{
  // TODO(Yadunund): Do we need a token at the node level? Right now I have one
  // for cases where a node may spin up but does not have any publishers or subscriptions.
  // Liveliness token for the node.
  zc_owned_liveliness_token_t token;
};

///=============================================================================
class rmw_publisher_data_t : public EventsBase
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

};

///=============================================================================
struct rmw_wait_set_data_t
{
  std::condition_variable condition_variable;
  std::mutex condition_mutex;

  rmw_context_t * context;
};

///=============================================================================
// z_owned_closure_sample_t
void sub_data_handler(const z_sample_t * sample, void * sub_data);

struct saved_msg_data
{
  explicit saved_msg_data(zc_owned_payload_t p, uint64_t recv_ts, const uint8_t pub_gid[16]);

  zc_owned_payload_t payload;
  uint64_t recv_timestamp;
  uint8_t publisher_gid[16];
};

///=============================================================================
class rmw_subscription_data_t : public EventsBase
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


///=============================================================================
void service_data_handler(const z_query_t * query, void * service_data);

///=============================================================================
void client_data_handler(z_owned_reply_t * reply, void * client_data);

///=============================================================================
class ZenohQuery final
{
public:
  ZenohQuery(const z_query_t * query);

  ~ZenohQuery();

  const z_query_t get_query() const;

private:
  z_owned_query_t query_;
};

///=============================================================================
class rmw_service_data_t : public EventsBase
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

///=============================================================================
class ZenohReply final
{
public:
  ZenohReply(const z_owned_reply_t * reply);

  ~ZenohReply();

  std::optional<z_sample_t> get_sample() const;

private:
  z_owned_reply_t reply_;
};

///=============================================================================
class rmw_client_data_t : public EventsBase
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

  uint8_t client_guid[RMW_GID_STORAGE_SIZE];

  size_t get_next_sequence_number();

  void add_new_reply(std::unique_ptr<ZenohReply> reply);

  bool reply_queue_is_empty() const;

  void attach_condition(std::condition_variable * condition_variable);

  void detach_condition();

  std::unique_ptr<ZenohReply> pop_next_reply();

private:
  void notify();

  size_t sequence_number{1};
  std::mutex sequence_number_mutex;

  std::condition_variable * condition_{nullptr};
  std::mutex condition_mutex_;

  std::deque<std::unique_ptr<ZenohReply>> reply_queue_;
  mutable std::mutex reply_queue_mutex_;
};

#endif  // DETAIL__RMW_DATA_TYPES_HPP_
