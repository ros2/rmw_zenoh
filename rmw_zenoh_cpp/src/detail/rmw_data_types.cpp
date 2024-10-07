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

#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "liveliness_utils.hpp"
#include "logging_macros.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"

#include "attachment_helpers.hpp"
#include "rmw_data_types.hpp"

///=============================================================================
namespace rmw_zenoh_cpp
{
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
  const size_t hash = rmw_zenoh_cpp::hash_gid(request_id.writer_guid);

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
  const size_t hash = rmw_zenoh_cpp::hash_gid(request_id.writer_guid);

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
