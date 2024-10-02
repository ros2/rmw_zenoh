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

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "cdr.hpp"
#include "message_type_support.hpp"
#include "logging_macros.hpp"
#include "qos.hpp"
#include "zenoh_utils.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"

namespace rmw_zenoh_cpp
{
// TODO(yuyuan): SHM, make this configurable
#define SHM_BUF_OK_SIZE 2621440

///=============================================================================
std::shared_ptr<PublisherData> PublisherData::make(
  const z_loaned_session_t * session,
  const rmw_node_t * const node,
  liveliness::NodeInfo node_info,
  std::size_t node_id,
  std::size_t publisher_id,
  const std::string & topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile)
{
  auto pub_data = std::shared_ptr<PublisherData>(new PublisherData{});
  pub_data->rmw_node_ = node;
  generate_random_gid(pub_data->gid_);
  rmw_qos_profile_t adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = QoS::get().best_available_qos(
    node, topic_name.c_str(), &adapted_qos_profile, rmw_get_subscriptions_info_by_topic);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  const rosidl_type_hash_t * type_hash = type_support->get_type_hash_func(type_support);
  pub_data->type_support_impl_ = type_support->data;
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);
  pub_data->type_support_ = std::make_unique<MessageTypeSupport>(callbacks);

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
  pub_data->entity_ = liveliness::Entity::make(
    z_info_zid(session),
    std::to_string(node_id),
    std::to_string(publisher_id),
    liveliness::EntityType::Publisher,
    std::move(node_info),
    liveliness::TopicInfo{
      std::move(domain_id),
      topic_name,
      pub_data->type_support_->get_name(),
      type_hash_c_str,
      adapted_qos_profile}
  );
  if (pub_data->entity_ == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the publisher %s.",
      topic_name.c_str());
    return nullptr;
  }

  std::string topic_keyexpr = pub_data->entity_->topic_info()->topic_keyexpr_;
  z_view_keyexpr_t pub_ke;
  if (z_view_keyexpr_from_str(&pub_ke, topic_keyexpr.c_str()) != Z_OK) {
    RMW_SET_ERROR_MSG("unable to create zenoh keyexpr.");
    return nullptr;
  }

  // Create a Publication Cache if durability is transient_local.
  if (adapted_qos_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    ze_publication_cache_options_t pub_cache_opts;
    ze_publication_cache_options_default(&pub_cache_opts);
    pub_cache_opts.history = adapted_qos_profile.depth;
    pub_cache_opts.queryable_complete = true;
    // Set the queryable_prefix to the session id so that querying subscribers can specify this
    // session id to obtain latest data from this specific publication caches when querying over
    // the same keyexpression.
    // When such a prefix is added to the PublicationCache, it listens to queries with this extra
    // prefix (allowing to be queried in a unique way), but still replies with the original
    // publications' key expressions.
    std::string queryable_prefix = pub_data->entity_->zid();
    z_view_keyexpr_t prefix_ke;
    z_view_keyexpr_from_str(&prefix_ke, queryable_prefix.c_str());
    pub_cache_opts.queryable_prefix = z_loan(prefix_ke);

    ze_owned_publication_cache_t pub_cache;
    if (ze_declare_publication_cache(
        &pub_cache, session, z_loan(pub_ke), &pub_cache_opts))
    {
      RMW_SET_ERROR_MSG("unable to create zenoh publisher cache");
      return nullptr;
    }
    pub_data->pub_cache_ = pub_cache;
  }
  auto undeclare_z_publisher_cache = rcpputils::make_scope_exit(
    [pub_data]() {
      if (pub_data && pub_data->pub_cache_.has_value()) {
        z_drop(z_move(pub_data->pub_cache_.value()));
      }
    });

  // Set congestion_control to BLOCK if appropriate.
  z_publisher_options_t opts;
  z_publisher_options_default(&opts);
  opts.congestion_control = Z_CONGESTION_CONTROL_DROP;

  if (adapted_qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
    opts.reliability = Z_RELIABILITY_RELIABLE;

    if (adapted_qos_profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
      opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
    }
  }
  // TODO(clalancette): What happens if the key name is a valid but empty string?
  auto undeclare_z_publisher = rcpputils::make_scope_exit(
    [pub_data]() {
      z_undeclare_publisher(z_move(pub_data->pub_));
    });
  if (z_declare_publisher(
      &pub_data->pub_, session, z_loan(pub_ke), &opts) != Z_OK)
  {
    RMW_SET_ERROR_MSG("Unable to create Zenoh publisher.");
    return nullptr;
  }

  std::string liveliness_keyexpr = pub_data->entity_->liveliness_keyexpr();
  z_view_keyexpr_t liveliness_ke;
  z_view_keyexpr_from_str(&liveliness_ke, liveliness_keyexpr.c_str());
  auto free_token = rcpputils::make_scope_exit(
    [pub_data]() {
      if (pub_data != nullptr) {
        z_drop(z_move(pub_data->token_));
      }
    });
  if (zc_liveliness_declare_token(
      &pub_data->token_, session, z_loan(liveliness_ke),
      NULL) != Z_OK)
  {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the publisher.");
    return nullptr;
  }

  // Initialize the events manager.
  pub_data->events_mgr_ = std::make_shared<EventsManager>();

  free_token.cancel();
  undeclare_z_publisher_cache.cancel();
  undeclare_z_publisher.cancel();

  return pub_data;
}

///=============================================================================
PublisherData::PublisherData()
: sequence_number_(1),
  is_shutdown_(false)
{
  // Do nothing.
}

///=============================================================================
rmw_qos_profile_t PublisherData::adapted_qos_profile() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->topic_info()->qos_;
}

///=============================================================================
rmw_ret_t PublisherData::publish(
  const void * ros_message,
  std::optional<z_owned_shm_provider_t> & shm_provider)
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
  std::optional<z_owned_shm_mut_t> shmbuf = std::nullopt;
  auto always_free_shmbuf = rcpputils::make_scope_exit(
    [&shmbuf]() {
      if (shmbuf.has_value()) {
        z_drop(z_move(shmbuf.value()));
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
  if (shm_provider.has_value()) {
    RMW_ZENOH_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "SHM is enabled.");

    auto provider = shm_provider.value();
    z_buf_layout_alloc_result_t alloc;
    // TODO(yuyuan): SHM, configure this
    z_alloc_alignment_t alignment = {5};
    z_shm_provider_alloc_gc_defrag_blocking(&alloc, z_loan(provider), SHM_BUF_OK_SIZE, alignment);

    if (alloc.status == ZC_BUF_LAYOUT_ALLOC_STATUS_OK) {
      shmbuf = std::make_optional(alloc.buf);
      msg_bytes = reinterpret_cast<char *>(z_shm_mut_data_mut(z_loan_mut(alloc.buf)));
    } else {
      // TODO(Yadunund): Should we revert to regular allocation and not return an error?
      RMW_SET_ERROR_MSG("Failed to allocate a SHM buffer, even after GCing.");
      return RMW_RET_ERROR;
    }
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

  z_owned_bytes_t attachment;
  auto free_attachment = rcpputils::make_scope_exit(
    [&attachment]() {
      z_drop(z_move(attachment));
    });
  if (!create_map_and_set_sequence_num(&attachment, sequence_number_++, gid_)) {
    // create_map_and_set_sequence_num already set the error
    return RMW_RET_ERROR;
  }

  // The encoding is simply forwarded and is useful when key expressions in the
  // session use different encoding formats. In our case, all key expressions
  // will be encoded with CDR so it does not really matter.
  z_publisher_put_options_t options;
  z_publisher_put_options_default(&options);
  free_attachment.cancel();
  options.attachment = z_move(attachment);

  z_owned_bytes_t payload;
  if (shmbuf.has_value()) {
    z_bytes_serialize_from_shm_mut(&payload, z_move(shmbuf.value()));
  } else {
    z_bytes_serialize_from_buf(&payload, reinterpret_cast<const uint8_t *>(msg_bytes), data_length);
  }

  if (z_publisher_put(z_loan(pub_), z_move(payload), &options) != Z_OK) {
    RMW_SET_ERROR_MSG("unable to publish message");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t PublisherData::publish_serialized_message(
  const rmw_serialized_message_t * serialized_message,
  std::optional<z_owned_shm_provider_t> & /*shm_provider*/)
{
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  rmw_zenoh_cpp::Cdr ser(buffer);
  if (!ser.get_cdr().jump(serialized_message->buffer_length)) {
    RMW_SET_ERROR_MSG("cannot correctly set serialized buffer");
    return RMW_RET_ERROR;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  z_owned_bytes_t attachment;
  auto free_attachment = rcpputils::make_scope_exit(
    [&attachment]() {
      z_drop(z_move(attachment));
    });
  if (!create_map_and_set_sequence_num(&attachment, sequence_number_++, gid_)) {
    // create_map_and_set_sequence_num already set the error
    return RMW_RET_ERROR;
  }

  const size_t data_length = ser.get_serialized_data_length();

  // The encoding is simply forwarded and is useful when key expressions in the
  // session use different encoding formats. In our case, all key expressions
  // will be encoded with CDR so it does not really matter.
  z_publisher_put_options_t options;
  z_publisher_put_options_default(&options);
  free_attachment.cancel();
  options.attachment = z_move(attachment);

  z_owned_bytes_t payload;
  z_bytes_serialize_from_buf(&payload, serialized_message->buffer, data_length);

  if (z_publisher_put(z_loan(pub_), z_move(payload), &options) != Z_OK) {
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

  // Unregister this node from the ROS graph.
  zc_liveliness_undeclare_token(z_move(token_));
  if (pub_cache_.has_value()) {
    z_drop(z_move(pub_cache_.value()));
  }
  z_undeclare_publisher(z_move(pub_));

  is_shutdown_ = true;
  return RMW_RET_OK;
}

///=============================================================================
// Check if the Node is shutdown.
bool PublisherData::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}
}  // namespace rmw_zenoh_cpp
