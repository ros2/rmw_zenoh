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
#include <utility>

#include "liveliness_utils.hpp"
#include "logging_macros.hpp"
#include "rmw_data_types.hpp"

#include "rmw/impl/cpp/macros.hpp"


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
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_loan(this->keyexpr), &keystr);

    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Query queue depth of %ld reached, discarding oldest Query "
      "for service for %s",
      adapted_qos_profile.depth,
      z_string_data(z_loan(keystr)));
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
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_loan(this->keyexpr), &keystr);
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Reply queue depth of %ld reached, discarding oldest reply "
      "for client for %s",
      adapted_qos_profile.depth,
      z_string_data(z_loan(keystr)));
    reply_queue_.pop_front();
  }
  reply_queue_.emplace_back(std::move(reply));

  // Since we added new data, trigger user callback and guard condition if they
  // are available
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
// See the comment about the "num_in_flight" class variable in the
// rmw_client_data_t structure for the use of this method.
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
ZenohQuery::ZenohQuery(z_owned_query_t query)
{
  query_ = query;
}

///=============================================================================
ZenohQuery::~ZenohQuery()
{
  z_drop(z_move(query_));
}

///=============================================================================
const z_loaned_query_t * ZenohQuery::get_query() const
{
  return z_loan(query_);
}

//==============================================================================
void service_data_handler(z_loaned_query_t * query, void * data)
{
  z_view_string_t keystr;
  z_keyexpr_as_view_string(z_query_keyexpr(query), &keystr);

  rmw_service_data_t * service_data =
    static_cast<rmw_service_data_t *>(data);
  if (service_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain rmw_service_data_t from data for "
      "service for %s",
      z_string_data(z_loan(keystr)));
    return;
  }

  z_owned_query_t owned_query;
  z_query_clone(&owned_query, query);

  service_data->add_new_query(std::make_unique<ZenohQuery>(owned_query));
}

///=============================================================================
ZenohReply::ZenohReply(z_owned_reply_t reply)
{
  reply_ = reply;
}

///=============================================================================
ZenohReply::~ZenohReply()
{
  z_drop(z_move(reply_));
}

///=============================================================================
const z_loaned_reply_t * ZenohReply::get_reply() const
{
  return z_loan(reply_);
}

///=============================================================================
size_t rmw_client_data_t::get_next_sequence_number()
{
  std::lock_guard<std::mutex> lock(sequence_number_mutex_);
  return sequence_number_++;
}

//==============================================================================
void client_data_handler(z_loaned_reply_t * reply, void * data)
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

  if (z_reply_is_ok(reply)) {
    z_owned_reply_t owned_reply;
    z_reply_clone(&owned_reply, reply);
    client_data->add_new_reply(std::make_unique<ZenohReply>(owned_reply));
  } else {
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_loan(client_data->keyexpr), &keystr);
    const z_loaned_reply_err_t * err = z_reply_err(reply);
    const z_loaned_bytes_t * err_payload = z_reply_err_payload(err);

    z_owned_string_t err_str;
    z_bytes_to_string(err_payload, &err_str);
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "z_reply_is_ok returned False for keyexpr %s. Reason: %.*s",
      z_string_data(z_loan(keystr)), static_cast<int>(z_string_len(z_loan(err_str))),
      z_string_data(z_loan(err_str)));
    z_drop(z_move(err_str));
    return;
  }
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
