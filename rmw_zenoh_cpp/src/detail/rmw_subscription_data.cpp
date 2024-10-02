// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "rmw_subscription_data.hpp"

#include <fastcdr/FastBuffer.h>

#include <cinttypes>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <variant>

#include "attachment_helpers.hpp"
#include "cdr.hpp"
#include "identifier.hpp"
#include "rmw_context_impl_s.hpp"
#include "message_type_support.hpp"
#include "logging_macros.hpp"
#include "qos.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/impl/cpp/macros.hpp"

namespace rmw_zenoh_cpp
{
namespace
{
//==============================================================================
// TODO(Yadunund): Make this a class member and lambda capture weak_from_this()
// instead of passing a rawptr to SubscriptionData when we switch to zenoh-cpp.
void sub_data_handler(const z_sample_t * sample, void * data)
{
  z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
  auto drop_keystr = rcpputils::make_scope_exit(
    [&keystr]() {
      z_drop(z_move(keystr));
    });

  auto sub_data = static_cast<SubscriptionData *>(data);
  if (sub_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain SubscriptionData from data for %s.",
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
    std::make_unique<SubscriptionData::Message>(
      zc_sample_payload_rcinc(sample),
      sample->timestamp.time, pub_gid, sequence_number, source_timestamp), z_loan(keystr));
}
}  // namespace

///=============================================================================
SubscriptionData::Message::Message(
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
SubscriptionData::Message::~Message()
{
  z_drop(z_move(payload));
}

///=============================================================================
std::shared_ptr<SubscriptionData> SubscriptionData::make(
  z_session_t session,
  std::shared_ptr<GraphCache> graph_cache,
  const rmw_node_t * const node,
  liveliness::NodeInfo node_info,
  std::size_t node_id,
  std::size_t subscription_id,
  const std::string & topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile)
{
  auto sub_data = std::shared_ptr<SubscriptionData>(new SubscriptionData{});
  sub_data->rmw_node_ = node;
  rmw_qos_profile_t adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = QoS::get().best_available_qos(
    node, topic_name.c_str(), &adapted_qos_profile, rmw_get_publishers_info_by_topic);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  const rosidl_type_hash_t * type_hash = type_support->get_type_hash_func(type_support);
  sub_data->type_support_impl_ = type_support->data;
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);
  sub_data->type_support_ = std::make_unique<MessageTypeSupport>(callbacks);

  // Convert the type hash to a string so that it can be included in
  // the keyexpr.
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(
    type_hash,
    *allocator,
    &type_hash_c_str);
  if (RCUTILS_RET_BAD_ALLOC == stringify_ret) {
    RMW_SET_ERROR_MSG("Failed to allocate type_hash_c_str.");
    return nullptr;
  }
  auto free_type_hash_c_str = rcpputils::make_scope_exit(
    [&allocator, &type_hash_c_str]() {
      allocator->deallocate(type_hash_c_str, allocator->state);
    });

  // Everything above succeeded and is setup properly.  Now declare a subscriber
  // with Zenoh; after this, callbacks may come in at any time.
  std::size_t domain_id = node_info.domain_id_;
  sub_data->entity_ = liveliness::Entity::make(
    z_info_zid(session),
    std::to_string(node_id),
    std::to_string(subscription_id),
    liveliness::EntityType::Subscription,
    std::move(node_info),
    liveliness::TopicInfo{
      std::move(domain_id),
      topic_name,
      sub_data->type_support_->get_name(),
      type_hash_c_str,
      adapted_qos_profile}
  );
  if (sub_data->entity_ == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the subscription %s.",
      topic_name.c_str());
    return nullptr;
  }
  // TODO(Yadunund): Instead of passing a rawptr, rely on capturing weak_ptr<SubscriptionData>
  // in the closure callback once we switch to zenoh-cpp.
  z_owned_closure_sample_t callback = z_closure(sub_data_handler, nullptr, sub_data.get());
  z_owned_keyexpr_t keyexpr =
    z_keyexpr_new(sub_data->entity_->topic_info()->topic_keyexpr_.c_str());
  auto always_free_ros_keyexpr = rcpputils::make_scope_exit(
    [&keyexpr]() {
      z_keyexpr_drop(z_move(keyexpr));
    });
  if (!z_keyexpr_check(&keyexpr)) {
    RMW_SET_ERROR_MSG("unable to create zenoh keyexpr.");
    return nullptr;
  }
  // Instantiate the subscription with suitable options depending on the
  // adapted_qos_profile.
  // TODO(Yadunund): Rely on a separate function to return the sub
  // as we start supporting more qos settings.
  z_owned_str_t owned_key_str = z_keyexpr_to_string(z_loan(keyexpr));
  auto always_drop_keystr = rcpputils::make_scope_exit(
    [&owned_key_str]() {
      z_drop(z_move(owned_key_str));
    });

  if (adapted_qos_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    ze_querying_subscriber_options_t sub_options = ze_querying_subscriber_options_default();
    // Make the initial query to hit all the PublicationCaches, using a query_selector with
    // '*' in place of the queryable_prefix of each PublicationCache
    const std::string selector = "*/" +
      sub_data->entity_->topic_info()->topic_keyexpr_;
    sub_options.query_selector = z_keyexpr(selector.c_str());
    // Tell the PublicationCache's Queryable that the query accepts any key expression as a reply.
    // By default a query accepts only replies that matches its query selector.
    // This allows us to selectively query certain PublicationCaches when defining the
    // set_querying_subscriber_callback below.
    sub_options.query_accept_replies = ZCU_REPLY_KEYEXPR_ANY;
    // As this initial query is now using a "*", the query target is not COMPLETE.
    sub_options.query_target = Z_QUERY_TARGET_ALL;
    // We set consolidation to none as we need to receive transient local messages
    // from a number of publishers. Eg: To receive TF data published over /tf_static
    // by various publishers.
    sub_options.query_consolidation = z_query_consolidation_none();
    if (adapted_qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      sub_options.reliability = Z_RELIABILITY_RELIABLE;
    }
    sub_data->sub_ = ze_declare_querying_subscriber(
      session,
      z_loan(keyexpr),
      z_move(callback),
      &sub_options
    );
    if (!z_check(std::get<ze_owned_querying_subscriber_t>(sub_data->sub_))) {
      RMW_SET_ERROR_MSG("unable to create zenoh subscription");
      return nullptr;
    }
    // Register the querying subscriber with the graph cache to get latest
    // messages from publishers that were discovered after their first publication.
    std::weak_ptr<SubscriptionData> data_wp = sub_data;
    graph_cache->set_querying_subscriber_callback(
      sub_data->entity_->topic_info().value().topic_keyexpr_,
      sub_data->entity_->guid(),
      [data_wp](const std::string & queryable_prefix) -> void
      {
        auto sub_data = data_wp.lock();
        if (sub_data == nullptr) {
          RMW_ZENOH_LOG_ERROR_NAMED(
            "rmw_zenoh_cpp",
            "Unable to lock weak_ptr<SubscriptionData> within querying subscription callback."
          );
          return;
        }
        std::lock_guard<std::mutex> lock(sub_data->mutex_);

        const std::string selector = queryable_prefix +
        "/" +
        sub_data->entity_->topic_info().value().topic_keyexpr_;
        RMW_ZENOH_LOG_DEBUG_NAMED(
          "rmw_zenoh_cpp",
          "QueryingSubscriberCallback triggered over %s.",
          selector.c_str()
        );
        z_get_options_t opts = z_get_options_default();
        opts.timeout_ms = std::numeric_limits<uint64_t>::max();
        opts.consolidation = z_query_consolidation_latest();
        opts.accept_replies = ZCU_REPLY_KEYEXPR_ANY;
        ze_querying_subscriber_get(
          z_loan(std::get<ze_owned_querying_subscriber_t>(sub_data->sub_)),
          z_keyexpr(selector.c_str()),
          &opts
        );
      }
    );
  } else {
    // Create a regular subscriber for all other durability settings.
    z_subscriber_options_t sub_options = z_subscriber_options_default();
    if (qos_profile->reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      sub_options.reliability = Z_RELIABILITY_RELIABLE;
    }
    sub_data->sub_ = z_declare_subscriber(
      session,
      z_loan(keyexpr),
      z_move(callback),
      &sub_options
    );
    if (!z_check(std::get<z_owned_subscriber_t>(sub_data->sub_))) {
      RMW_SET_ERROR_MSG("unable to create zenoh subscription");
      return nullptr;
    }
  }
  sub_data->graph_cache_ = graph_cache;

  auto undeclare_z_sub = rcpputils::make_scope_exit(
    [data = sub_data]() {
      z_owned_subscriber_t * sub = std::get_if<z_owned_subscriber_t>(&data->sub_);
      if (sub == nullptr || z_undeclare_subscriber(sub)) {
        RMW_SET_ERROR_MSG("failed to undeclare sub");
      } else {
        ze_owned_querying_subscriber_t * querying_sub =
        std::get_if<ze_owned_querying_subscriber_t>(&data->sub_);
        if (querying_sub == nullptr || ze_undeclare_querying_subscriber(querying_sub)) {
          RMW_SET_ERROR_MSG("failed to undeclare sub");
        }
      }
    });

  // Publish to the graph that a new subscription is in town.
  sub_data->token_ = zc_liveliness_declare_token(
    session,
    z_keyexpr(sub_data->entity_->liveliness_keyexpr().c_str()),
    NULL
  );
  auto free_token = rcpputils::make_scope_exit(
    [data = sub_data]() {
      if (data != nullptr) {
        z_drop(z_move(data->token_));
      }
    });
  if (!z_check(sub_data->token_)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the subscription.");
    return nullptr;
  }

  // Initialize the events manager.
  sub_data->events_mgr_ = std::make_shared<EventsManager>();

  free_token.cancel();
  undeclare_z_sub.cancel();

  return sub_data;
}

///=============================================================================
SubscriptionData::SubscriptionData()
: total_messages_lost_(0),
  wait_set_data_(nullptr),
  is_shutdown_(false)
{
  // Do nothing.
}

///=============================================================================
rmw_qos_profile_t SubscriptionData::adapted_qos_profile() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->topic_info()->qos_;
}


///=============================================================================
std::size_t SubscriptionData::guid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->guid();
}

///=============================================================================
liveliness::TopicInfo SubscriptionData::topic_info() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->topic_info().value();
}

///=============================================================================
bool SubscriptionData::liveliness_is_valid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return zc_liveliness_token_check(&token_);
}

///=============================================================================
std::shared_ptr<EventsManager> SubscriptionData::events_mgr() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return events_mgr_;
}

///=============================================================================
SubscriptionData::~SubscriptionData()
{
  const rmw_ret_t ret = this->shutdown();
  if (ret != RMW_RET_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Error destructing publisher /%s.",
      entity_->topic_info().value().name_.c_str()
    );
  }
}

///=============================================================================
rmw_ret_t SubscriptionData::shutdown()
{
  rmw_ret_t ret = RMW_RET_OK;
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return ret;
  }

  // Unregister this node from the ROS graph.
  zc_liveliness_undeclare_token(z_move(token_));

  z_owned_subscriber_t * sub = std::get_if<z_owned_subscriber_t>(&sub_);
  if (sub != nullptr) {
    if (z_undeclare_subscriber(sub)) {
      RMW_SET_ERROR_MSG("failed to undeclare sub.");
      ret = RMW_RET_ERROR;
    }
  } else {
    ze_owned_querying_subscriber_t * querying_sub =
      std::get_if<ze_owned_querying_subscriber_t>(&sub_);
    if (querying_sub == nullptr || ze_undeclare_querying_subscriber(querying_sub)) {
      RMW_SET_ERROR_MSG("failed to undeclare sub.");
      ret = RMW_RET_ERROR;
    }
  }

  is_shutdown_ = true;
  return RMW_RET_OK;
}

///=============================================================================
bool SubscriptionData::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}

///=============================================================================
bool SubscriptionData::queue_has_data_and_attach_condition_if_not(
  rmw_wait_set_data_t * wait_set_data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!message_queue_.empty()) {
    return true;
  }

  wait_set_data_ = wait_set_data;

  return false;
}

///=============================================================================
bool SubscriptionData::detach_condition_and_queue_is_empty()
{
  std::lock_guard<std::mutex> lock(mutex_);
  wait_set_data_ = nullptr;

  return message_queue_.empty();
}

///=============================================================================
rmw_ret_t SubscriptionData::take_one_message(
  void * ros_message,
  rmw_message_info_t * message_info,
  bool * taken)
{
  *taken = false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || message_queue_.empty()) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  std::unique_ptr<Message> msg_data = std::move(message_queue_.front());
  message_queue_.pop_front();

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(const_cast<uint8_t *>(msg_data->payload.payload.start)),
    msg_data->payload.payload.len);

  // Object that serializes the data
  rmw_zenoh_cpp::Cdr deser(fastbuffer);
  if (!type_support_->deserialize_ros_message(
      deser.get_cdr(),
      ros_message,
      type_support_impl_))
  {
    RMW_SET_ERROR_MSG("could not deserialize ROS message");
    return RMW_RET_ERROR;
  }

  if (message_info != nullptr) {
    message_info->source_timestamp = msg_data->source_timestamp;
    message_info->received_timestamp = msg_data->recv_timestamp;
    message_info->publication_sequence_number = msg_data->sequence_number;
    // TODO(clalancette): fill in reception_sequence_number
    message_info->reception_sequence_number = 0;
    message_info->publisher_gid.implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
    memcpy(message_info->publisher_gid.data, msg_data->publisher_gid, RMW_GID_STORAGE_SIZE);
    message_info->from_intra_process = false;
  }

  *taken = true;

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t SubscriptionData::take_serialized_message(
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info)
{
  *taken = false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || message_queue_.empty()) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  std::unique_ptr<Message> msg_data = std::move(message_queue_.front());
  message_queue_.pop_front();

  if (serialized_message->buffer_capacity < msg_data->payload.payload.len) {
    rmw_ret_t ret =
      rmw_serialized_message_resize(serialized_message, msg_data->payload.payload.len);
    if (ret != RMW_RET_OK) {
      return ret;  // Error message already set
    }
  }
  serialized_message->buffer_length = msg_data->payload.payload.len;
  memcpy(
    serialized_message->buffer, msg_data->payload.payload.start,
    msg_data->payload.payload.len);

  *taken = true;

  if (message_info != nullptr) {
    message_info->source_timestamp = msg_data->source_timestamp;
    message_info->received_timestamp = msg_data->recv_timestamp;
    message_info->publication_sequence_number = msg_data->sequence_number;
    // TODO(clalancette): fill in reception_sequence_number
    message_info->reception_sequence_number = 0;
    message_info->publisher_gid.implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
    memcpy(message_info->publisher_gid.data, msg_data->publisher_gid, RMW_GID_STORAGE_SIZE);
    message_info->from_intra_process = false;
  }

  return RMW_RET_OK;
}

///=============================================================================
void SubscriptionData::add_new_message(
  std::unique_ptr<SubscriptionData::Message> msg, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }
  const rmw_qos_profile_t adapted_qos_profile = entity_->topic_info().value().qos_;
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
      std::unique_ptr<Message> old = std::move(message_queue_.front());
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
      events_mgr_->add_new_event(
        ZENOH_EVENT_MESSAGE_LOST,
        std::move(event_status));
    }
  }
  // Always update the last known sequence number for the publisher.
  last_known_published_msg_[gid_hash] = msg->sequence_number;

  message_queue_.emplace_back(std::move(msg));

  // Since we added new data, trigger user callback and guard condition if they are available
  data_callback_mgr_.trigger_callback();
  if (wait_set_data_ != nullptr) {
    wait_set_data_->triggered = true;
    wait_set_data_->condition_variable.notify_one();
  }
}

//==============================================================================
void SubscriptionData::set_on_new_message_callback(
  rmw_event_callback_t callback,
  const void * user_data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_callback_mgr_.set_callback(user_data, callback);
}

//==============================================================================
std::shared_ptr<GraphCache> SubscriptionData::graph_cache() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_cache_;
}

}  // namespace rmw_zenoh_cpp
