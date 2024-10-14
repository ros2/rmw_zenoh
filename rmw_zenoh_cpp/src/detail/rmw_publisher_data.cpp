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

#include "rmw_publisher_data.hpp"

#include <fastcdr/FastBuffer.h>

#include <cinttypes>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "cdr.hpp"
#include "rmw_context_impl_s.hpp"
#include "message_type_support.hpp"
#include "logging_macros.hpp"
#include "qos.hpp"
#include "zenoh_utils.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/impl/cpp/macros.hpp"

#include "tracetools/tracetools.h"

namespace rmw_zenoh_cpp
{
///=============================================================================
std::shared_ptr<PublisherData> PublisherData::make(
  z_session_t session,
  const rmw_publisher_t * const rmw_publisher,
  const rmw_node_t * const node,
  liveliness::NodeInfo node_info,
  std::size_t node_id,
  std::size_t publisher_id,
  const std::string & topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile)
{
  rmw_qos_profile_t adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = QoS::get().best_available_qos(
    node, topic_name.c_str(), &adapted_qos_profile, rmw_get_subscriptions_info_by_topic);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  const rosidl_type_hash_t * type_hash = type_support->get_type_hash_func(type_support);
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);
  auto message_type_support = std::make_unique<MessageTypeSupport>(callbacks);

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
  auto always_free_type_hash_c_str = rcpputils::make_scope_exit(
    [&allocator, &type_hash_c_str]() {
      allocator->deallocate(type_hash_c_str, allocator->state);
    });

  std::size_t domain_id = node_info.domain_id_;
  auto entity = liveliness::Entity::make(
    z_info_zid(session),
    std::to_string(node_id),
    std::to_string(publisher_id),
    liveliness::EntityType::Publisher,
    std::move(node_info),
    liveliness::TopicInfo{
      std::move(domain_id),
      topic_name,
      message_type_support->get_name(),
      type_hash_c_str,
      adapted_qos_profile}
  );
  if (entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the publisher %s.",
      topic_name.c_str());
    return nullptr;
  }
  z_owned_keyexpr_t keyexpr = z_keyexpr_new(
    entity->topic_info()->topic_keyexpr_.c_str());
  auto always_free_ros_keyexpr = rcpputils::make_scope_exit(
    [&keyexpr]() {
      z_keyexpr_drop(z_move(keyexpr));
    });
  if (!z_keyexpr_check(&keyexpr)) {
    RMW_SET_ERROR_MSG("unable to create zenoh keyexpr.");
    return nullptr;
  }

  // Create a Publication Cache if durability is transient_local.
  std::optional<ze_owned_publication_cache_t> pub_cache = std::nullopt;
  if (adapted_qos_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    ze_publication_cache_options_t pub_cache_opts = ze_publication_cache_options_default();
    pub_cache_opts.history = adapted_qos_profile.depth;
    pub_cache_opts.queryable_complete = true;
    // Set the queryable_prefix to the session id so that querying subscribers can specify this
    // session id to obtain latest data from this specific publication caches when querying over
    // the same keyexpression.
    // When such a prefix is added to the PublicationCache, it listens to queries with this extra
    // prefix (allowing to be queried in a unique way), but still replies with the original
    // publications' key expressions.
    z_owned_keyexpr_t queryable_prefix = z_keyexpr_new(entity->zid().c_str());
    auto always_free_queryable_prefix = rcpputils::make_scope_exit(
      [&queryable_prefix]() {
        z_keyexpr_drop(z_move(queryable_prefix));
      });
    pub_cache_opts.queryable_prefix = z_loan(queryable_prefix);
    pub_cache = ze_declare_publication_cache(
      session,
      z_loan(keyexpr),
      &pub_cache_opts
    );
    if (!pub_cache.has_value() || !z_check(pub_cache.value())) {
      RMW_SET_ERROR_MSG("unable to create zenoh publisher cache");
      return nullptr;
    }
  }
  auto undeclare_z_publisher_cache = rcpputils::make_scope_exit(
    [&pub_cache]() {
      if (pub_cache.has_value()) {
        z_drop(z_move(pub_cache.value()));
      }
    });

  // Set congestion_control to BLOCK if appropriate.
  z_publisher_options_t opts = z_publisher_options_default();
  opts.congestion_control = Z_CONGESTION_CONTROL_DROP;
  if (adapted_qos_profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
    adapted_qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE)
  {
    opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
  }
  // TODO(clalancette): What happens if the key name is a valid but empty string?
  z_owned_publisher_t pub = z_declare_publisher(
    session,
    z_loan(keyexpr),
    &opts
  );
  if (!z_check(pub)) {
    RMW_SET_ERROR_MSG("Unable to create Zenoh publisher.");
    return nullptr;
  }
  auto undeclare_z_publisher = rcpputils::make_scope_exit(
    [&pub]() {
      z_undeclare_publisher(z_move(pub));
    });

  zc_owned_liveliness_token_t token = zc_liveliness_declare_token(
    session,
    z_keyexpr(entity->liveliness_keyexpr().c_str()),
    NULL
  );
  auto free_token = rcpputils::make_scope_exit(
    [&token]() {
      z_drop(z_move(token));
    });
  if (!z_check(token)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the publisher.");
    return nullptr;
  }

  free_token.cancel();
  undeclare_z_publisher_cache.cancel();
  undeclare_z_publisher.cancel();

  return std::shared_ptr<PublisherData>(
    new PublisherData{
      rmw_publisher,
      node,
      std::move(entity),
      std::move(pub),
      std::move(pub_cache),
      std::move(token),
      type_support->data,
      std::move(message_type_support)
    });
}

///=============================================================================
PublisherData::PublisherData(
  const rmw_publisher_t * const rmw_publisher,
  const rmw_node_t * rmw_node,
  std::shared_ptr<liveliness::Entity> entity,
  z_owned_publisher_t pub,
  std::optional<ze_owned_publication_cache_t> pub_cache,
  zc_owned_liveliness_token_t token,
  const void * type_support_impl,
  std::unique_ptr<MessageTypeSupport> type_support)
: rmw_publisher_(rmw_publisher),
  rmw_node_(rmw_node),
  entity_(std::move(entity)),
  pub_(std::move(pub)),
  pub_cache_(std::move(pub_cache)),
  token_(std::move(token)),
  type_support_impl_(type_support_impl),
  type_support_(std::move(type_support)),
  sequence_number_(1),
  is_shutdown_(false)
{
  generate_random_gid(gid_);
  events_mgr_ = std::make_shared<EventsManager>();
}

///=============================================================================
rmw_ret_t PublisherData::publish(
  const void * ros_message,
  std::optional<zc_owned_shm_manager_t> & shm_manager)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    RMW_SET_ERROR_MSG("Unable to publish as the publisher has been shutdown.");
    return RMW_RET_ERROR;
  }

  // Serialize data.
  size_t max_data_length = type_support_->get_estimated_serialized_size(
    ros_message,
    type_support_impl_);

  // To store serialized message byte array.
  char * msg_bytes = nullptr;
  std::optional<zc_owned_shmbuf_t> shmbuf = std::nullopt;
  auto always_free_shmbuf = rcpputils::make_scope_exit(
    [&shmbuf]() {
      if (shmbuf.has_value()) {
        zc_shmbuf_drop(&shmbuf.value());
      }
    });

  rcutils_allocator_t * allocator = &rmw_node_->context->options.allocator;

  auto always_free_msg_bytes = rcpputils::make_scope_exit(
    [&msg_bytes, allocator, &shmbuf]() {
      if (msg_bytes && !shmbuf.has_value()) {
        allocator->deallocate(msg_bytes, allocator->state);
      }
    });

  // Get memory from SHM buffer if available.
  if (shm_manager.has_value() &&
    zc_shm_manager_check(&shm_manager.value()))
  {
    shmbuf = zc_shm_alloc(
      &shm_manager.value(),
      max_data_length);
    if (!z_check(shmbuf.value())) {
      zc_shm_gc(&shm_manager.value());
      shmbuf = zc_shm_alloc(&shm_manager.value(), max_data_length);
      if (!z_check(shmbuf.value())) {
        // TODO(Yadunund): Should we revert to regular allocation and not return an error?
        RMW_SET_ERROR_MSG("Failed to allocate a SHM buffer, even after GCing.");
        return RMW_RET_ERROR;
      }
    }
    msg_bytes = reinterpret_cast<char *>(zc_shmbuf_ptr(&shmbuf.value()));
  } else {
    // Get memory from the allocator.
    msg_bytes = static_cast<char *>(allocator->allocate(max_data_length, allocator->state));
    RMW_CHECK_FOR_NULL_WITH_MSG(
      msg_bytes, "bytes for message is null", return RMW_RET_BAD_ALLOC);
  }

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(msg_bytes, max_data_length);

  // Object that serializes the data
  rmw_zenoh_cpp::Cdr ser(fastbuffer);
  if (!type_support_->serialize_ros_message(
      ros_message,
      ser.get_cdr(),
      type_support_impl_))
  {
    RMW_SET_ERROR_MSG("could not serialize ROS message");
    return RMW_RET_ERROR;
  }

  const size_t data_length = ser.get_serialized_data_length();

  int64_t source_timestamp = 0;
  z_owned_bytes_map_t map =
    create_map_and_set_sequence_num(
    sequence_number_++,
    [this](z_owned_bytes_map_t * map, const char * key)
    {
      // Mutex already locked.
      z_bytes_t gid_bytes;
      gid_bytes.len = RMW_GID_STORAGE_SIZE;
      gid_bytes.start = gid_;
      z_bytes_map_insert_by_copy(map, z_bytes_new(key), gid_bytes);
    },
    &source_timestamp);
  if (!z_check(map)) {
    // create_map_and_set_sequence_num already set the error
    return RMW_RET_ERROR;
  }
  auto always_free_attachment_map = rcpputils::make_scope_exit(
    [&map]() {
      z_bytes_map_drop(z_move(map));
    });

  int ret;
  // The encoding is simply forwarded and is useful when key expressions in the
  // session use different encoding formats. In our case, all key expressions
  // will be encoded with CDR so it does not really matter.
  z_publisher_put_options_t options = z_publisher_put_options_default();
  options.attachment = z_bytes_map_as_attachment(&map);

  TRACETOOLS_TRACEPOINT(
    rmw_publish, static_cast<const void *>(rmw_publisher_), ros_message, source_timestamp);
  if (shmbuf.has_value()) {
    zc_shmbuf_set_length(&shmbuf.value(), data_length);
    zc_owned_payload_t payload = zc_shmbuf_into_payload(z_move(shmbuf.value()));
    ret = zc_publisher_put_owned(z_loan(pub_), z_move(payload), &options);
  } else {
    // Returns 0 if success.
    ret = z_publisher_put(
      z_loan(pub_),
      reinterpret_cast<const uint8_t *>(msg_bytes),
      data_length,
      &options);
  }
  if (ret) {
    RMW_SET_ERROR_MSG("unable to publish message");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t PublisherData::publish_serialized_message(
  const rmw_serialized_message_t * serialized_message,
  std::optional<zc_owned_shm_manager_t> & /*shm_manager*/)
{
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  rmw_zenoh_cpp::Cdr ser(buffer);
  if (!ser.get_cdr().jump(serialized_message->buffer_length)) {
    RMW_SET_ERROR_MSG("cannot correctly set serialized buffer");
    return RMW_RET_ERROR;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  int64_t source_timestamp = 0;
  z_owned_bytes_map_t map = rmw_zenoh_cpp::create_map_and_set_sequence_num(
    sequence_number_++,
    [this](z_owned_bytes_map_t * map, const char * key)
    {
      // Mutex already locked.
      z_bytes_t gid_bytes;
      gid_bytes.len = RMW_GID_STORAGE_SIZE;
      gid_bytes.start = gid_;
      z_bytes_map_insert_by_copy(map, z_bytes_new(key), gid_bytes);
    },
    &source_timestamp);

  if (!z_check(map)) {
    // create_map_and_set_sequence_num already set the error
    return RMW_RET_ERROR;
  }
  auto free_attachment_map = rcpputils::make_scope_exit(
    [&map]() {
      z_bytes_map_drop(z_move(map));
    });

  const size_t data_length = ser.get_serialized_data_length();

  // The encoding is simply forwarded and is useful when key expressions in the
  // session use different encoding formats. In our case, all key expressions
  // will be encoded with CDR so it does not really matter.
  z_publisher_put_options_t options = z_publisher_put_options_default();
  options.attachment = z_bytes_map_as_attachment(&map);

  TRACETOOLS_TRACEPOINT(
    rmw_publish, static_cast<const void *>(rmw_publisher_), serialized_message, source_timestamp);
  // Returns 0 if success.
  int8_t ret = z_publisher_put(
    z_loan(pub_),
    serialized_message->buffer,
    data_length,
    &options);

  if (ret) {
    RMW_SET_ERROR_MSG("unable to publish message");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

///=============================================================================
std::size_t PublisherData::guid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->guid();
}

///=============================================================================
liveliness::TopicInfo PublisherData::topic_info() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->topic_info().value();
}

///=============================================================================
void PublisherData::copy_gid(rmw_gid_t * gid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  memcpy(gid->data, gid_, RMW_GID_STORAGE_SIZE);
}

///=============================================================================
bool PublisherData::liveliness_is_valid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return zc_liveliness_token_check(&token_);
}

///=============================================================================
std::shared_ptr<EventsManager> PublisherData::events_mgr() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return events_mgr_;
}

///=============================================================================
PublisherData::~PublisherData()
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
rmw_ret_t PublisherData::shutdown()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return RMW_RET_OK;
  }

  // Unregister this publisher from the ROS graph.
  zc_liveliness_undeclare_token(z_move(token_));
  if (pub_cache_.has_value()) {
    z_drop(z_move(pub_cache_.value()));
  }
  z_undeclare_publisher(z_move(pub_));

  is_shutdown_ = true;
  return RMW_RET_OK;
}

///=============================================================================
bool PublisherData::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}
}  // namespace rmw_zenoh_cpp
