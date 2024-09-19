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

#include <zenoh.h>

#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "logging_macros.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"

#include "attachment_helpers.hpp"
#include "rmw_data_types.hpp"
#include "zenoh_config.hpp"
#include "zenoh_router_check.hpp"


///=============================================================================
namespace
{
size_t hash_gid(const uint8_t * gid)
{
  std::stringstream hash_str;
  hash_str << std::hex;
  size_t i = 0;
  for (; i < (RMW_GID_STORAGE_SIZE - 1); i++) {
    hash_str << static_cast<int>(gid[i]);
  }
  return std::hash<std::string>{}(hash_str.str());
}

///=============================================================================
size_t hash_gid(const rmw_request_id_t & request_id)
{
  return hash_gid(request_id.writer_guid);
}
}  // namespace

///=============================================================================
size_t rmw_context_impl_s::get_next_entity_id()
{
  return next_entity_id_++;
}

///=============================================================================
void rmw_context_impl_s::async_check_router()
{
  const std::optional<uint64_t> configured_connection_attempts =
    rmw_zenoh_cpp::zenoh_router_check_attempts();
  if (configured_connection_attempts.has_value()) {
    router_check_thread_ = std::thread(
      [this](std::optional<uint64_t> configured_connection_attempts, z_session_t session)
      {
        uint64_t connection_attempts = 0;
        rmw_ret_t ret = RMW_RET_TIMEOUT;
        // Retry until the connection is successful.
        while (ret != RMW_RET_OK && connection_attempts < configured_connection_attempts.value()) {
          if ((ret = rmw_zenoh_cpp::zenoh_router_check(session)) != RMW_RET_OK) {
            ++connection_attempts;
          }
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (ret != RMW_RET_OK) {
          RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
            "Unable to connect to a Zenoh router after %zu retries.",
            configured_connection_attempts.value());
          // Return without setting router_is_available_ to true.
          return;
        }
        // The router check has succeeded so we set router_is_available_ to true.
        std::lock_guard<std::mutex> lock(router_check_mutex_);
        router_is_available_ = true;
        router_check_cv_.notify_all();
      },
      configured_connection_attempts,
      z_loan(session)
    );
  }
  else {
    // If we skip the router check, directly set available_ to true.
    std::lock_guard<std::mutex> lock(router_check_mutex_);
    router_is_available_ = true;
  }
}

///=============================================================================
bool rmw_context_impl_s::wait_for_router()
{
  // Wait until the cv is notified  or router check fails.
  std::unique_lock<std::mutex> lock(router_check_mutex_);
  router_check_cv_.wait(lock, [this]{ return router_is_available_ == true;});
  return router_is_available_;
}

///=============================================================================
rmw_context_impl_s::~rmw_context_impl_s()
{
  if (router_check_thread_.joinable()) {
    router_check_thread_.join();
  }
}


namespace rmw_zenoh_cpp
{
///=============================================================================
saved_msg_data::saved_msg_data(
  zc_owned_payload_t p,
  uint64_t recv_ts,
  const uint8_t pub_gid[RMW_GID_STORAGE_SIZE],
  int64_t seqnum,
  int64_t source_ts)
: payload(p), recv_timestamp(recv_ts), sequence_number(seqnum), source_timestamp(source_ts)
{
  memcpy(publisher_gid, pub_gid, RMW_GID_STORAGE_SIZE);
}

///=============================================================================
saved_msg_data::~saved_msg_data()
{
  z_drop(z_move(payload));
}

///=============================================================================
size_t rmw_publisher_data_t::get_next_sequence_number()
{
  std::lock_guard<std::mutex> lock(sequence_number_mutex_);
  return sequence_number_++;
}

///=============================================================================
bool rmw_subscription_data_t::queue_has_data_and_attach_condition_if_not(
  rmw_wait_set_data_t * wait_set_data)
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  if (!message_queue_.empty()) {
    return true;
  }

  wait_set_data_ = wait_set_data;

  return false;
}

///=============================================================================
void rmw_subscription_data_t::notify()
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  if (wait_set_data_ != nullptr) {
    std::lock_guard<std::mutex> wait_set_lock(wait_set_data_->condition_mutex);
    wait_set_data_->triggered = true;
    wait_set_data_->condition_variable.notify_one();
  }
}

///=============================================================================
bool rmw_subscription_data_t::detach_condition_and_queue_is_empty()
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  wait_set_data_ = nullptr;

  return message_queue_.empty();
}

///=============================================================================
std::unique_ptr<saved_msg_data> rmw_subscription_data_t::pop_next_message()
{
  std::lock_guard<std::mutex> lock(message_queue_mutex_);

  if (message_queue_.empty()) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return nullptr;
  }

  std::unique_ptr<rmw_zenoh_cpp::saved_msg_data> msg_data = std::move(message_queue_.front());
  message_queue_.pop_front();

  return msg_data;
}

///=============================================================================
void rmw_subscription_data_t::add_new_message(
  std::unique_ptr<saved_msg_data> msg, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(message_queue_mutex_);

  if (adapted_qos_profile.history != RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
    message_queue_.size() >= adapted_qos_profile.depth)
  {
    // Log warning if message is discarded due to hitting the queue depth
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Message queue depth of %ld reached, discarding oldest message "
      "for subscription for %s",
      adapted_qos_profile.depth,
      topic_name.c_str());

    // If the adapted_qos_profile.depth is 0, the std::move command below will result
    // in UB and the z_drop will segfault. We explicitly set the depth to a minimum of 1
    // in rmw_create_subscription() but to be safe, we only attempt to discard from the
    // queue if it is non-empty.
    if (!message_queue_.empty()) {
      std::unique_ptr<saved_msg_data> old = std::move(message_queue_.front());
      message_queue_.pop_front();
    }
  }

  // Check for messages lost if the new sequence number is not monotonically increasing.
  const size_t gid_hash = hash_gid(msg->publisher_gid);
  auto last_known_pub_it = last_known_published_msg_.find(gid_hash);
  if (last_known_pub_it != last_known_published_msg_.end()) {
    const int64_t seq_increment = std::abs(msg->sequence_number - last_known_pub_it->second);
    if (seq_increment > 1) {
      const size_t num_msg_lost = seq_increment - 1;
      total_messages_lost_ += num_msg_lost;
      auto event_status = std::make_unique<rmw_zenoh_event_status_t>();
      event_status->total_count_change = num_msg_lost;
      event_status->total_count = total_messages_lost_;
      events_mgr.add_new_event(
        ZENOH_EVENT_MESSAGE_LOST,
        std::move(event_status));
    }
  }
  // Always update the last known sequence number for the publisher
  last_known_published_msg_[gid_hash] = msg->sequence_number;

  message_queue_.emplace_back(std::move(msg));

  // Since we added new data, trigger user callback and guard condition if they are available
  data_callback_mgr.trigger_callback();
  notify();
}

///=============================================================================
bool rmw_service_data_t::queue_has_data_and_attach_condition_if_not(
  rmw_wait_set_data_t * wait_set_data)
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  if (!query_queue_.empty()) {
    return true;
  }
  wait_set_data_ = wait_set_data;

  return false;
}

///=============================================================================
bool rmw_service_data_t::detach_condition_and_queue_is_empty()
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  wait_set_data_ = nullptr;

  return query_queue_.empty();
}

///=============================================================================
std::unique_ptr<ZenohQuery> rmw_service_data_t::pop_next_query()
{
  std::lock_guard<std::mutex> lock(query_queue_mutex_);
  if (query_queue_.empty()) {
    return nullptr;
  }

  std::unique_ptr<ZenohQuery> query = std::move(query_queue_.front());
  query_queue_.pop_front();

  return query;
}

///=============================================================================
void rmw_service_data_t::notify()
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  if (wait_set_data_ != nullptr) {
    std::lock_guard<std::mutex> wait_set_lock(wait_set_data_->condition_mutex);
    wait_set_data_->triggered = true;
    wait_set_data_->condition_variable.notify_one();
  }
}

///=============================================================================
void rmw_service_data_t::add_new_query(std::unique_ptr<ZenohQuery> query)
{
  std::lock_guard<std::mutex> lock(query_queue_mutex_);
  if (adapted_qos_profile.history != RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
    query_queue_.size() >= adapted_qos_profile.depth)
  {
    // Log warning if message is discarded due to hitting the queue depth
    z_owned_str_t keystr = z_keyexpr_to_string(z_loan(this->keyexpr));
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Query queue depth of %ld reached, discarding oldest Query "
      "for service for %s",
      adapted_qos_profile.depth,
      z_loan(keystr));
    z_drop(z_move(keystr));
    query_queue_.pop_front();
  }
  query_queue_.emplace_back(std::move(query));

  // Since we added new data, trigger user callback and guard condition if they are available
  data_callback_mgr.trigger_callback();
  notify();
}

///=============================================================================
bool rmw_service_data_t::add_to_query_map(
  const rmw_request_id_t & request_id, std::unique_ptr<ZenohQuery> query)
{
  size_t hash = hash_gid(request_id);

  std::lock_guard<std::mutex> lock(sequence_to_query_map_mutex_);

  std::unordered_map<size_t, SequenceToQuery>::iterator it = sequence_to_query_map_.find(hash);

  if (it == sequence_to_query_map_.end()) {
    SequenceToQuery stq;

    sequence_to_query_map_.insert(std::make_pair(hash, std::move(stq)));

    it = sequence_to_query_map_.find(hash);
  } else {
    // Client already in the map

    if (it->second.find(request_id.sequence_number) != it->second.end()) {
      return false;
    }
  }

  it->second.insert(std::make_pair(request_id.sequence_number, std::move(query)));

  return true;
}

///=============================================================================
std::unique_ptr<ZenohQuery> rmw_service_data_t::take_from_query_map(
  const rmw_request_id_t & request_id)
{
  size_t hash = hash_gid(request_id);

  std::lock_guard<std::mutex> lock(sequence_to_query_map_mutex_);

  std::unordered_map<size_t, SequenceToQuery>::iterator it = sequence_to_query_map_.find(hash);

  if (it == sequence_to_query_map_.end()) {
    return nullptr;
  }

  SequenceToQuery::iterator query_it = it->second.find(request_id.sequence_number);

  if (query_it == it->second.end()) {
    return nullptr;
  }

  std::unique_ptr<ZenohQuery> query = std::move(query_it->second);
  it->second.erase(query_it);

  if (sequence_to_query_map_[hash].size() == 0) {
    sequence_to_query_map_.erase(hash);
  }

  return query;
}

///=============================================================================
void rmw_client_data_t::notify()
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  if (wait_set_data_ != nullptr) {
    std::lock_guard<std::mutex> wait_set_lock(wait_set_data_->condition_mutex);
    wait_set_data_->triggered = true;
    wait_set_data_->condition_variable.notify_one();
  }
}

///=============================================================================
void rmw_client_data_t::add_new_reply(std::unique_ptr<ZenohReply> reply)
{
  std::lock_guard<std::mutex> lock(reply_queue_mutex_);
  if (adapted_qos_profile.history != RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
    reply_queue_.size() >= adapted_qos_profile.depth)
  {
    // Log warning if message is discarded due to hitting the queue depth
    z_owned_str_t keystr = z_keyexpr_to_string(z_loan(this->keyexpr));
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Reply queue depth of %ld reached, discarding oldest reply "
      "for client for %s",
      adapted_qos_profile.depth,
      z_loan(keystr));
    z_drop(z_move(keystr));
    reply_queue_.pop_front();
  }
  reply_queue_.emplace_back(std::move(reply));

  // Since we added new data, trigger user callback and guard condition if they are available
  data_callback_mgr.trigger_callback();
  notify();
}

///=============================================================================
bool rmw_client_data_t::queue_has_data_and_attach_condition_if_not(
  rmw_wait_set_data_t * wait_set_data)
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  if (!reply_queue_.empty()) {
    return true;
  }
  wait_set_data_ = wait_set_data;

  return false;
}

///=============================================================================
bool rmw_client_data_t::detach_condition_and_queue_is_empty()
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  wait_set_data_ = nullptr;

  return reply_queue_.empty();
}

///=============================================================================
std::unique_ptr<ZenohReply> rmw_client_data_t::pop_next_reply()
{
  std::lock_guard<std::mutex> lock(reply_queue_mutex_);

  if (reply_queue_.empty()) {
    return nullptr;
  }

  std::unique_ptr<ZenohReply> latest_reply = std::move(reply_queue_.front());
  reply_queue_.pop_front();

  return latest_reply;
}

//==============================================================================
// See the comment about the "num_in_flight" class variable in the rmw_client_data_t class
// for the use of this method.
void rmw_client_data_t::increment_in_flight_callbacks()
{
  std::lock_guard<std::mutex> lock(in_flight_mutex_);
  num_in_flight_++;
}

//==============================================================================
// See the comment about the "num_in_flight" class variable in the rmw_client_data_t class
// for the use of this method.
bool rmw_client_data_t::shutdown_and_query_in_flight()
{
  std::lock_guard<std::mutex> lock(in_flight_mutex_);
  is_shutdown_ = true;

  return num_in_flight_ > 0;
}

//==============================================================================
// See the comment about the "num_in_flight" class variable in the rmw_client_data_t structure
// for the use of this method.
bool rmw_client_data_t::decrement_queries_in_flight_and_is_shutdown(bool & queries_in_flight)
{
  std::lock_guard<std::mutex> lock(in_flight_mutex_);
  queries_in_flight = --num_in_flight_ > 0;
  return is_shutdown_;
}

bool rmw_client_data_t::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(in_flight_mutex_);
  return is_shutdown_;
}

//==============================================================================
void sub_data_handler(
  const z_sample_t * sample,
  void * data)
{
  z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
  auto drop_keystr = rcpputils::make_scope_exit(
    [&keystr]() {
      z_drop(z_move(keystr));
    });

  auto sub_data = static_cast<rmw_subscription_data_t *>(data);
  if (sub_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain rmw_subscription_data_t from data for "
      "subscription for %s",
      z_loan(keystr)
    );
    return;
  }

  uint8_t pub_gid[RMW_GID_STORAGE_SIZE];
  if (!get_gid_from_attachment(&sample->attachment, pub_gid)) {
    // We failed to get the GID from the attachment.  While this isn't fatal,
    // it is unusual and so we should report it.
    memset(pub_gid, 0, RMW_GID_STORAGE_SIZE);
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain publisher GID from the attachment.");
  }

  int64_t sequence_number = get_int64_from_attachment(&sample->attachment, "sequence_number");
  if (sequence_number < 0) {
    // We failed to get the sequence number from the attachment.  While this
    // isn't fatal, it is unusual and so we should report it.
    sequence_number = 0;
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Unable to obtain sequence number from the attachment.");
  }

  int64_t source_timestamp = get_int64_from_attachment(&sample->attachment, "source_timestamp");
  if (source_timestamp < 0) {
    // We failed to get the source timestamp from the attachment.  While this
    // isn't fatal, it is unusual and so we should report it.
    source_timestamp = 0;
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Unable to obtain sequence number from the attachment.");
  }

  sub_data->add_new_message(
    std::make_unique<saved_msg_data>(
      zc_sample_payload_rcinc(sample),
      sample->timestamp.time, pub_gid, sequence_number, source_timestamp), z_loan(keystr));
}

///=============================================================================
ZenohQuery::ZenohQuery(const z_query_t * query)
{
  query_ = z_query_clone(query);
}

///=============================================================================
ZenohQuery::~ZenohQuery()
{
  z_drop(z_move(query_));
}

///=============================================================================
const z_query_t ZenohQuery::get_query() const
{
  return z_query_loan(&query_);
}

//==============================================================================
void service_data_handler(const z_query_t * query, void * data)
{
  z_owned_str_t keystr = z_keyexpr_to_string(z_query_keyexpr(query));
  auto drop_keystr = rcpputils::make_scope_exit(
    [&keystr]() {
      z_drop(z_move(keystr));
    });

  rmw_service_data_t * service_data =
    static_cast<rmw_service_data_t *>(data);
  if (service_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain rmw_service_data_t from data for "
      "service for %s",
      z_loan(keystr)
    );
    return;
  }

  service_data->add_new_query(std::make_unique<ZenohQuery>(query));
}

///=============================================================================
ZenohReply::ZenohReply(const z_owned_reply_t * reply)
{
  reply_ = *reply;
}

///=============================================================================
ZenohReply::~ZenohReply()
{
  z_reply_drop(z_move(reply_));
}

///=============================================================================
std::optional<z_sample_t> ZenohReply::get_sample() const
{
  if (z_reply_is_ok(&reply_)) {
    return z_reply_ok(&reply_);
  }

  return std::nullopt;
}

///=============================================================================
size_t rmw_client_data_t::get_next_sequence_number()
{
  std::lock_guard<std::mutex> lock(sequence_number_mutex_);
  return sequence_number_++;
}

//==============================================================================
void client_data_handler(z_owned_reply_t * reply, void * data)
{
  auto client_data = static_cast<rmw_client_data_t *>(data);
  if (client_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain client_data_t "
    );
    return;
  }

  // See the comment about the "num_in_flight" class variable in the rmw_client_data_t class for
  // why we need to do this.
  if (client_data->is_shutdown()) {
    return;
  }

  if (!z_reply_check(reply)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "z_reply_check returned False"
    );
    return;
  }
  if (!z_reply_is_ok(reply)) {
    z_owned_str_t keystr = z_keyexpr_to_string(z_loan(client_data->keyexpr));
    z_value_t err = z_reply_err(reply);
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "z_reply_is_ok returned False for keyexpr %s. Reason: %.*s",
      z_loan(keystr),
      (int)err.payload.len,
      err.payload.start);
    z_drop(z_move(keystr));

    return;
  }

  client_data->add_new_reply(std::make_unique<ZenohReply>(reply));
  // Since we took ownership of the reply, null it out here
  *reply = z_reply_null();
}

void client_data_drop(void * data)
{
  auto client_data = static_cast<rmw_client_data_t *>(data);
  if (client_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain client_data_t "
    );
    return;
  }

  // See the comment about the "num_in_flight" class variable in the rmw_client_data_t class for
  // why we need to do this.
  bool queries_in_flight = false;
  bool is_shutdown = client_data->decrement_queries_in_flight_and_is_shutdown(queries_in_flight);

  if (is_shutdown) {
    if (!queries_in_flight) {
      RMW_TRY_DESTRUCTOR(client_data->~rmw_client_data_t(), rmw_client_data_t, );
      client_data->context->options.allocator.deallocate(
        client_data, client_data->context->options.allocator.state);
    }
  }
}

}  // namespace rmw_zenoh_cpp
