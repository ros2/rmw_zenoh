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

#include <array>
#include <cinttypes>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>
#include <cstdint>

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
// TODO(yuyuan): SHM, make this configurable
#define SHM_BUF_OK_SIZE 2621440

// Period (ms) of heartbeats sent for detection of lost samples
// by a RELIABLE + TRANSIENT_LOCAL Publisher
#define SAMPLE_MISS_DETECTION_HEARTBEAT_PERIOD 500

///=============================================================================
std::shared_ptr<PublisherData> PublisherData::make(
  std::shared_ptr<zenoh::Session> session,
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
    // rosidl_stringify_type_hash already set the error
    return nullptr;
  }
  auto always_free_type_hash_c_str = rcpputils::make_scope_exit(
    [&allocator, &type_hash_c_str]() {
      allocator->deallocate(type_hash_c_str, allocator->state);
    });

  std::size_t domain_id = node_info.domain_id_;
  auto entity = liveliness::Entity::make(
    session->get_zid(),
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

  using AdvancedPublisherOptions = zenoh::ext::SessionExt::AdvancedPublisherOptions;
  using SampleMissDetectionOptions = AdvancedPublisherOptions::SampleMissDetectionOptions;
  auto adv_pub_opts = AdvancedPublisherOptions::create_default();

  if (adapted_qos_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    // Allow this publisher to be detected through liveliness.
    adv_pub_opts.publisher_detection = true;
    adv_pub_opts.cache = AdvancedPublisherOptions::CacheOptions::create_default();
    adv_pub_opts.cache->max_samples = adapted_qos_profile.depth;
    if (adapted_qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      // If RELIABLE + TRANSIENT_LOCAL activate sample miss detection for subscriber
      // to detect missed samples and retrieve those from the Publisher cache.
      // HeartbeatSporadic is used to prevent excessive background traffic
      adv_pub_opts.sample_miss_detection = SampleMissDetectionOptions{};
      adv_pub_opts.sample_miss_detection->heartbeat =
        SampleMissDetectionOptions::HeartbeatSporadic{
        SAMPLE_MISS_DETECTION_HEARTBEAT_PERIOD};
    }
  }

  zenoh::KeyExpr pub_ke(entity->topic_info()->topic_keyexpr_);
  // Set congestion_control to BLOCK if appropriate.
  auto pub_opts = zenoh::Session::PublisherOptions::create_default();
  pub_opts.congestion_control = Z_CONGESTION_CONTROL_DROP;
  if (adapted_qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
    pub_opts.reliability = Z_RELIABILITY_RELIABLE;
    if (adapted_qos_profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
      pub_opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
    }
  } else {
    pub_opts.reliability = Z_RELIABILITY_BEST_EFFORT;
  }
  adv_pub_opts.publisher_options = pub_opts;

  zenoh::ZResult result;
  auto adv_pub = session->ext().declare_advanced_publisher(
    pub_ke, std::move(adv_pub_opts), &result);
  if (result != Z_OK) {
    RMW_SET_ERROR_MSG("unable to create zenoh publisher cache");
    return nullptr;
  }

  if (result != Z_OK) {
    RMW_SET_ERROR_MSG("Unable to create Zenoh publisher.");
    return nullptr;
  }

  std::string liveliness_keyexpr = entity->liveliness_keyexpr();
  auto token = session->liveliness_declare_token(
    zenoh::KeyExpr(liveliness_keyexpr),
    zenoh::Session::LivelinessDeclarationOptions::create_default(),
    &result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the publisher.");
    return nullptr;
  }

  return std::shared_ptr<PublisherData>(
    new PublisherData{
      rmw_publisher,
      node,
      std::move(entity),
      std::move(session),
      std::move(adv_pub),
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
  std::shared_ptr<zenoh::Session> sess,
  zenoh::ext::AdvancedPublisher pub,
  zenoh::LivelinessToken token,
  const void * type_support_impl,
  std::unique_ptr<MessageTypeSupport> type_support)
: rmw_publisher_(rmw_publisher),
  rmw_node_(rmw_node),
  entity_(std::move(entity)),
  sess_(std::move(sess)),
  pub_(std::move(pub)),
  token_(std::move(token)),
  type_support_impl_(type_support_impl),
  type_support_(std::move(type_support)),
  sequence_number_(1),
  is_shutdown_(false)
{
  events_mgr_ = std::make_shared<EventsManager>();
}

///=============================================================================
rmw_ret_t PublisherData::publish(
  const void * ros_message,
  const std::shared_ptr<ShmContext> shm)
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
  uint8_t * msg_bytes = nullptr;

  rmw_context_impl_s * context_impl =
    static_cast<rmw_context_impl_s *>(rmw_node_->data);
  if (context_impl == nullptr) {
    RMW_SET_ERROR_MSG("Unable to cast rmw_node->data into rmw_context_impl_s.");
    return RMW_RET_ERROR;
  }
  rcutils_allocator_t * allocator = &rmw_node_->context->options.allocator;

  // Optional shared memory buffer
  std::optional<zenoh::ZShmMut> shm_buf = std::nullopt;
  // Optional buffer reused for serialization from the buffer pool
  std::optional<BufferPool::Buffer> pool_buf = std::nullopt;

  auto always_free_msg_bytes = rcpputils::make_scope_exit(
    [&msg_bytes, allocator, &shm_buf, &pool_buf]() {
      if (!msg_bytes || shm_buf.has_value() || (pool_buf.has_value() && pool_buf.value().data)) {
        return;
      }
      allocator->deallocate(msg_bytes, allocator->state);
    });

  // Get memory from SHM buffer if available.
  if (shm && max_data_length >= shm->msgsize_threshold) {
    RMW_ZENOH_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "SHM is enabled.");

    auto alloc_result = shm->shm_provider.alloc_gc_defrag(max_data_length);

    if (std::holds_alternative<zenoh::ZShmMut>(alloc_result)) {
      auto && buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
      msg_bytes = reinterpret_cast<uint8_t *>(buf.data());
      shm_buf = std::make_optional(std::move(buf));
    } else {
      // Print a warning and revert to regular allocation
      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "Failed to allocate a SHM buffer, fallback to non-SHM");
    }
  }

  if (!shm_buf.has_value()) {
    // Try to get memory from the serialization buffer pool.
    pool_buf = context_impl->serialization_buffer_pool()->allocate(max_data_length);
    if (pool_buf.has_value() && pool_buf.value().data) {
      msg_bytes = pool_buf->data;
    } else {
      void * data = allocator->allocate(max_data_length, allocator->state);
      RMW_CHECK_FOR_NULL_WITH_MSG(
        data, "failed to allocate serialization buffer", return RMW_RET_BAD_ALLOC);
      msg_bytes = static_cast<uint8_t *>(data);
    }
  }

  RMW_CHECK_FOR_NULL_WITH_MSG(
    msg_bytes, "bytes for message is null", return RMW_RET_BAD_ALLOC);


  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char *>(msg_bytes), max_data_length);

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

  // The encoding is simply forwarded and is useful when key expressions in the
  // session use different encoding formats. In our case, all key expressions
  // will be encoded with CDR so it does not really matter.
  zenoh::ZResult result;
  int64_t source_timestamp = rmw_zenoh_cpp::get_system_time_in_ns();
  auto opts = zenoh::ext::AdvancedPublisher::PutOptions::create_default();
  opts.put_options.attachment = rmw_zenoh_cpp::AttachmentData(
    sequence_number_++, source_timestamp, entity_->copy_gid()).serialize_to_zbytes();

  zenoh::Bytes payload;
  if (shm_buf.has_value()) {
    payload = zenoh::Bytes(std::move(*shm_buf));
  } else if (pool_buf.has_value() && pool_buf.value().data) {
    auto deleter = [buffer_pool = context_impl->serialization_buffer_pool(),
        buffer = pool_buf](uint8_t *){
        buffer_pool->deallocate(buffer.value());
      };
    payload = zenoh::Bytes(msg_bytes, data_length, deleter);
  } else {
    auto deleter = [msg_bytes, allocator](uint8_t *) {
        allocator->deallocate(msg_bytes, allocator->state);
      };
    payload = zenoh::Bytes(msg_bytes, data_length, deleter);
  }
  // The delete responsibility has been handed over to zenoh::Bytes now
  always_free_msg_bytes.cancel();

  TRACETOOLS_TRACEPOINT(
    rmw_publish, static_cast<const void *>(rmw_publisher_), ros_message, source_timestamp);
  pub_.put(std::move(payload), std::move(opts), &result);
  if (result != Z_OK) {
    if (result == Z_ESESSION_CLOSED) {
      RMW_ZENOH_LOG_WARN_NAMED(
        "rmw_zenoh_cpp",
        "unable to publish message since the zenoh session is closed");
    } else {
      RMW_SET_ERROR_MSG("unable to publish message");
      return RMW_RET_ERROR;
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t PublisherData::publish_serialized_message(
  const rmw_serialized_message_t * serialized_message,
  const std::shared_ptr<ShmContext> shm)
{
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  rmw_zenoh_cpp::Cdr ser(buffer);
  if (!ser.get_cdr().jump(serialized_message->buffer_length)) {
    RMW_SET_ERROR_MSG("cannot correctly set serialized buffer");
    return RMW_RET_ERROR;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  const size_t data_length = ser.get_serialized_data_length();
  // The encoding is simply forwarded and is useful when key expressions in the
  // session use different encoding formats. In our case, all key expressions
  // will be encoded with CDR so it does not really matter.
  zenoh::ZResult result;
  int64_t source_timestamp = rmw_zenoh_cpp::get_system_time_in_ns();
  auto opts = zenoh::ext::AdvancedPublisher::PutOptions::create_default();
  opts.put_options.attachment = rmw_zenoh_cpp::AttachmentData(
    sequence_number_++, source_timestamp, entity_->copy_gid()).serialize_to_zbytes();

  // Get memory from SHM buffer if available.
  if (shm && data_length >= shm->msgsize_threshold) {
    RMW_ZENOH_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "SHM is enabled.");

    auto alloc_result = shm->shm_provider.alloc_gc_defrag(data_length);

    if (std::holds_alternative<zenoh::ZShmMut>(alloc_result)) {
      auto && buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
      auto msg_bytes = reinterpret_cast<char *>(buf.data());
      memcpy(msg_bytes, serialized_message->buffer, data_length);
      zenoh::Bytes payload(std::move(buf));

      TRACETOOLS_TRACEPOINT(
        rmw_publish, static_cast<const void *>(rmw_publisher_), serialized_message,
        source_timestamp);

      pub_.put(std::move(payload), std::move(opts), &result);
    } else {
      // Print a warning and revert to regular allocation
      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "Failed to allocate a SHM buffer, fallback to non-SHM");

      // TODO(yellowhatter): split the whole publish method onto shm and non-shm versions
      std::vector<uint8_t> raw_image(
        serialized_message->buffer,
        serialized_message->buffer + data_length);
      zenoh::Bytes payload(raw_image);

      TRACETOOLS_TRACEPOINT(
        rmw_publish, static_cast<const void *>(rmw_publisher_), serialized_message,
          source_timestamp);

      pub_.put(std::move(payload), std::move(opts), &result);
    }
  } else {
    std::vector<uint8_t> raw_image(
      serialized_message->buffer,
      serialized_message->buffer + data_length);
    zenoh::Bytes payload(raw_image);

    TRACETOOLS_TRACEPOINT(
      rmw_publish, static_cast<const void *>(rmw_publisher_), serialized_message, source_timestamp);

    pub_.put(std::move(payload), std::move(opts), &result);
  }

  if (result != Z_OK) {
    if (result == Z_ESESSION_CLOSED) {
      RMW_ZENOH_LOG_WARN_NAMED(
        "rmw_zenoh_cpp",
        "unable to publish message since the zenoh session is closed");
    } else {
      RMW_SET_ERROR_MSG("unable to publish message");
      return RMW_RET_ERROR;
    }
  }
  return RMW_RET_OK;
}

///=============================================================================
std::size_t PublisherData::gid_hash() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->gid_hash();
}

///=============================================================================
liveliness::TopicInfo PublisherData::topic_info() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->topic_info().value();
}

std::array<uint8_t, RMW_GID_STORAGE_SIZE> PublisherData::copy_gid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->copy_gid();
}

///=============================================================================
bool PublisherData::liveliness_is_valid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  // The z_check function is now internal in zenoh-1.0.0 so we assume
  // the liveliness token is still initialized as long as this entity has
  // not been shutdown.
  return !is_shutdown_;
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
  zenoh::ZResult result;
  std::move(token_).value().undeclare(&result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to undeclare the liveliness token for topic '%s'",
      entity_->topic_info().value().name_.c_str());
    return RMW_RET_ERROR;
  }
  std::move(pub_).undeclare(&result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to undeclare the publisher for topic '%s'",
      entity_->topic_info().value().name_.c_str());
    return RMW_RET_ERROR;
  }

  sess_.reset();
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
