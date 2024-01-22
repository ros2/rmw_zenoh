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

#include <cstring>
#include <mutex>
#include <optional>
#include <utility>

#include "rcpputils/scope_exit.hpp"
#include "rcutils/logging_macros.h"

#include "rmw_data_types.hpp"

///==============================================================================
saved_msg_data::saved_msg_data(zc_owned_payload_t p, uint64_t recv_ts, const uint8_t pub_gid[16])
: payload(p), recv_timestamp(recv_ts)
{
  memcpy(publisher_gid, pub_gid, 16);
}

void rmw_subscription_data_t::attach_condition(std::condition_variable * condition_variable)
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  condition_ = condition_variable;
}

void rmw_subscription_data_t::notify()
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  if (condition_ != nullptr) {
    condition_->notify_one();
  }
}

void rmw_subscription_data_t::detach_condition()
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  condition_ = nullptr;
}

bool rmw_subscription_data_t::message_queue_is_empty() const
{
  std::lock_guard<std::mutex> lock(message_queue_mutex_);
  return message_queue_.empty();
}

std::unique_ptr<saved_msg_data> rmw_subscription_data_t::pop_next_message()
{
  std::lock_guard<std::mutex> lock(message_queue_mutex_);

  if (message_queue_.empty()) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return nullptr;
  }

  std::unique_ptr<saved_msg_data> msg_data = std::move(message_queue_.front());
  message_queue_.pop_front();

  return msg_data;
}

void rmw_subscription_data_t::add_new_message(
  std::unique_ptr<saved_msg_data> msg, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(message_queue_mutex_);

  if (message_queue_.size() >= queue_depth) {
    // Log warning if message is discarded due to hitting the queue depth
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "Message queue depth of %ld reached, discarding oldest message "
      "for subscription for %s",
      queue_depth,
      topic_name.c_str());

    std::unique_ptr<saved_msg_data> old = std::move(message_queue_.front());
    z_drop(z_move(old->payload));
    message_queue_.pop_front();
  }

  message_queue_.emplace_back(std::move(msg));

  // Since we added new data, trigger the guard condition if it is available
  notify();
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
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain rmw_subscription_data_t from data for "
      "subscription for %s",
      z_loan(keystr)
    );
    return;
  }

  sub_data->add_new_message(
    std::make_unique<saved_msg_data>(
      zc_sample_payload_rcinc(sample),
      sample->timestamp.time, sample->timestamp.id.id), z_loan(keystr));
}

ZenohQuery::ZenohQuery(const z_query_t * query)
{
  query_ = z_query_clone(query);
}

ZenohQuery::~ZenohQuery()
{
  z_drop(z_move(query_));
}

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

  rmw_service_data_t * service_data = static_cast<rmw_service_data_t *>(data);
  if (service_data == nullptr) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain rmw_service_data_t from data for "
      "service for %s",
      z_loan(keystr)
    );
    return;
  }

  // Get the query parameters and payload
  {
    std::lock_guard<std::mutex> lock(service_data->query_queue_mutex);
    service_data->query_queue.emplace_back(std::make_unique<ZenohQuery>(query));
  }
  {
    // Since we added new data, trigger the guard condition if it is available
    std::lock_guard<std::mutex> internal_lock(service_data->internal_mutex);
    if (service_data->condition != nullptr) {
      service_data->condition->notify_one();
    }
  }
}

ZenohReply::ZenohReply(const z_owned_reply_t * reply)
{
  reply_ = *reply;
}

ZenohReply::~ZenohReply()
{
  z_reply_drop(z_move(reply_));
}

std::optional<z_sample_t> ZenohReply::get_sample() const
{
  if (z_reply_is_ok(&reply_)) {
    return z_reply_ok(&reply_);
  }

  return std::nullopt;
}

size_t rmw_client_data_t::get_next_sequence_number()
{
  std::lock_guard<std::mutex> lock(sequence_number_mutex);
  return sequence_number++;
}

//==============================================================================
void client_data_handler(z_owned_reply_t * reply, void * data)
{
  auto client_data = static_cast<rmw_client_data_t *>(data);
  if (client_data == nullptr) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain client_data_t "
    );
    return;
  }
  if (!z_reply_check(reply)) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "z_reply_check returned False"
    );
    return;
  }
  if (!z_reply_is_ok(reply)) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "z_reply_is_ok returned False"
    );
    return;
  }
  {
    std::lock_guard<std::mutex> msg_lock(client_data->replies_mutex);
    // Take ownership of the reply.
    client_data->replies.emplace_back(std::make_unique<ZenohReply>(reply));
    *reply = z_reply_null();
  }
  {
    std::lock_guard<std::mutex> internal_lock(client_data->internal_mutex);
    if (client_data->condition != nullptr) {
      client_data->condition->notify_one();
    }
  }
}
