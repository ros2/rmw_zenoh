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
#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>
#include <cstdint>

#include "cdr.hpp"

#include "rosidl_buffer_backend_registry/backend_utils.hpp"
#include "buffer_backend_context.hpp"
#include "buffer_endpoint_helpers.hpp"
#include "identifier.hpp"
#include "rmw_context_impl_s.hpp"
#include "message_type_support.hpp"
#include "logging_macros.hpp"
#include "qos.hpp"
#include "zenoh_utils.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rosidl_runtime_c/type_hash.h"

#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

#include "tracetools/tracetools.h"

namespace rmw_zenoh_cpp
{

using buffer_endpoint_helpers::entity_type_to_string;
using buffer_endpoint_helpers::gid_array_to_hex;
using buffer_endpoint_helpers::gid_to_hex;
using buffer_endpoint_helpers::is_cpu_only_backend_metadata;
using buffer_endpoint_helpers::make_accelerated_key;
using buffer_endpoint_helpers::make_cpu_group_key;

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

  const bool has_buffer_fields = callbacks->has_buffer_fields;
  // Buffer backends do not support transient-local durability. Those publishers
  // use only the base Zenoh publisher, which provides the required history cache.
  const bool use_buffer_endpoints = has_buffer_fields &&
    adapted_qos_profile.durability != RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED("rmw_zenoh_cpp",
    "[PublisherData::make] Creating publisher for topic '%s', "
    "type name '%s', has_buffer_fields: '%d', use_buffer_endpoints: '%d'",
    topic_name.c_str(), message_type_support->get_name(), has_buffer_fields,
    use_buffer_endpoints);

  // Get installed backends metadata if buffer endpoints are enabled.
  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(node->context->impl);
  std::unordered_map<std::string, std::string> backend_metadata;
  if (use_buffer_endpoints) {
    auto * backend_ctx = context_impl->buffer_backend_context();
    if (backend_ctx) {
      backend_metadata = rosidl_buffer_backend_registry::get_all_backend_metadata(
        backend_ctx->backend_instances);
    }
    // CPU serialization is always implicitly supported by buffer-aware publishers.
    if (backend_metadata.find("cpu") == backend_metadata.end()) {
      backend_metadata["cpu"] = "";
    }
  }

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
      adapted_qos_profile,
      use_buffer_endpoints ? std::make_optional(backend_metadata) : std::nullopt}
  );
  if (entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the publisher %s.",
      topic_name.c_str());
    return nullptr;
  }

  zenoh::ZResult result;
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

  auto pub_data = std::shared_ptr<PublisherData>(
    new PublisherData{
      rmw_publisher,
      node,
      std::move(entity),
      std::move(session),
      std::move(token),
      type_support->data,
      std::move(message_type_support),
      use_buffer_endpoints,
      backend_metadata
    });

  // Create the base publisher through the same path as every endpoint,
  // stored in endpoints_ under the topic key expression.
  const std::string base_key = pub_data->entity_->topic_info()->topic_keyexpr_;
  auto base_endpoint = pub_data->create_publisher_endpoint(base_key, /*buffer_aware=*/false);
  if (!base_endpoint) {
    RMW_SET_ERROR_MSG("Unable to create Zenoh publisher.");
    return nullptr;
  }
  pub_data->base_endpoint_ = base_endpoint.get();
  pub_data->endpoints_[base_key] = std::move(base_endpoint);

  if (use_buffer_endpoints) {
    pub_data->local_endpoint_info_ =
      build_endpoint_info_from_entity(*pub_data->entity_, RMW_ENDPOINT_PUBLISHER);

    auto * backend_ctx = context_impl->buffer_backend_context();
    if (backend_ctx) {
      rosidl_buffer_backend_registry::notify_endpoint_created(
        backend_ctx->backend_instances, pub_data->local_endpoint_info_.info);
    }

    // Eagerly create the CPU-group publisher endpoint.  All CPU-only
    // subscribers share this single channel instead of requiring individual
    // peer-to-peer endpoints -- mirrors rmw_fastrtps_cpp's eager CPU
    // DataWriter creation.
    const std::string cpu_group_key = make_cpu_group_key(
      pub_data->entity_->topic_info()->topic_keyexpr_);
    auto cpu_endpoint = pub_data->create_publisher_endpoint(cpu_group_key);
    if (!cpu_endpoint) {
      RMW_ZENOH_ROSIDL_BUFFER_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Failed to eagerly create CPU-group publisher endpoint for topic '%s'",
        topic_name.c_str());
      return nullptr;
    }
    pub_data->endpoints_[cpu_group_key] = std::move(cpu_endpoint);
  }

  // Register discovery callback for Buffer-aware publishers
  if (use_buffer_endpoints) {
    pub_data->graph_cache_ = context_impl->graph_cache();
    if (pub_data->graph_cache_ != nullptr) {
      std::weak_ptr<PublisherData> weak_pub_data = pub_data;
      pub_data->graph_cache_->register_subscriber_discovery_callback(
        topic_name,
        pub_data->gid_hash(),
        [weak_pub_data](const liveliness::Entity & entity) {
          if (auto pd = weak_pub_data.lock()) {
            pd->on_subscriber_discovered(entity);
          }
        });

      // Manually add this local publisher to the graph cache so local subscribers can discover it
      // Liveliness events from the same session don't trigger graph updates automatically
      pub_data->graph_cache_->parse_put(pub_data->entity_->liveliness_keyexpr(), false);
    }
  }

  return pub_data;
}

///=============================================================================
PublisherData::PublisherData(
  const rmw_publisher_t * const rmw_publisher,
  const rmw_node_t * rmw_node,
  std::shared_ptr<liveliness::Entity> entity,
  std::shared_ptr<zenoh::Session> sess,
  zenoh::LivelinessToken token,
  const void * type_support_impl,
  std::unique_ptr<MessageTypeSupport> type_support,
  bool is_buffer_aware,
  std::unordered_map<std::string, std::string> backend_metadata)
: rmw_publisher_(rmw_publisher),
  rmw_node_(rmw_node),
  entity_(std::move(entity)),
  sess_(std::move(sess)),
  token_(std::move(token)),
  type_support_impl_(type_support_impl),
  type_support_(std::move(type_support)),
  sequence_number_(1),
  is_shutdown_(false),
  is_buffer_aware_(is_buffer_aware),
  backend_metadata_(std::move(backend_metadata))
{
  events_mgr_ = std::make_shared<EventsManager>();
}

///=============================================================================
// Helper function for buffer-aware publishing
rmw_ret_t PublisherData::publish_buffer_aware(
  const void * ros_message,
  ShmContext * shm)
{
  (void)shm;  // SHM not currently used for buffer-aware publishing

  rmw_context_impl_t * ctx_impl = static_cast<rmw_context_impl_t *>(rmw_node_->context->impl);
  auto * backend_ctx = ctx_impl->buffer_backend_context();
  if (!backend_ctx) {
    RMW_SET_ERROR_MSG("Buffer-aware publish missing buffer backend context");
    return RMW_RET_ERROR;
  }

  RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[Publisher] Publishing buffer-aware message for topic '%s' (message type: %s)",
    entity_->topic_info()->name_.c_str(), entity_->topic_info()->type_.c_str());

  // For buffer-aware publishers, route to different endpoints based on discovered subscribers
  if (discovered_subscribers_.empty()) {
    // No subscribers yet, skip publish
    return RMW_RET_OK;
  }

  // Clear message caches
  for (auto & [key, ep] : endpoints_) {
    ep->cached_message.reset();
  }

  const std::string cpu_group_key =
    make_cpu_group_key(entity_->topic_info()->topic_keyexpr_);

  // Serialize and publish to each endpoint
  rmw_ret_t ret = RMW_RET_OK;

  auto cpu_endpoint_it = endpoints_.find(cpu_group_key);
  if (cpu_endpoint_it != endpoints_.end() &&
    cpu_endpoint_it->second &&
    !cpu_endpoint_it->second->target_subscribers.empty())
  {
    auto cpu_endpoint = cpu_endpoint_it->second;
    const size_t max_data_length = type_support_->get_estimated_serialized_size(
      ros_message, type_support_impl_);

    rcutils_allocator_t * allocator = &rmw_node_->context->options.allocator;
    void * data = allocator->allocate(max_data_length, allocator->state);
    if (!data) {
      RMW_SET_ERROR_MSG("failed to allocate CPU-group serialization buffer");
      return RMW_RET_BAD_ALLOC;
    }

    auto always_free_data = rcpputils::make_scope_exit(
      [data, allocator]() {
        allocator->deallocate(data, allocator->state);
      });

    uint8_t * msg_bytes = static_cast<uint8_t *>(data);
    eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char *>(msg_bytes), max_data_length);
    rmw_zenoh_cpp::Cdr ser(fastbuffer);
    if (!type_support_->serialize_ros_message(
        ros_message,
        ser.get_cdr(),
        type_support_impl_))
    {
      RMW_SET_ERROR_MSG("could not serialize ROS message for CPU-group publish");
      return RMW_RET_ERROR;
    }

    const size_t data_length = ser.get_serialized_data_length();
    auto deleter = [data, allocator](uint8_t *) {
        allocator->deallocate(data, allocator->state);
      };
    auto payload = zenoh::Bytes(msg_bytes, data_length, deleter);
    always_free_data.cancel();

    int64_t source_timestamp = rmw_zenoh_cpp::get_system_time_in_ns();
    auto gid = entity_->copy_gid();
    auto attachment_data = rmw_zenoh_cpp::AttachmentData(
      sequence_number_++, source_timestamp, gid);
    auto attachment_bytes = attachment_data.serialize_to_zbytes();

    zenoh::ext::AdvancedPublisher::PutOptions options =
      zenoh::ext::AdvancedPublisher::PutOptions::create_default();
    options.put_options.attachment = std::move(attachment_bytes);

    zenoh::ZResult result;
    if (cpu_endpoint->pub.has_value()) {
      cpu_endpoint->pub.value().put(std::move(payload), std::move(options), &result);
      if (result != Z_OK) {
        RMW_ZENOH_ROSIDL_BUFFER_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Failed to publish to CPU-group endpoint '%s'", cpu_group_key.c_str());
        ret = RMW_RET_ERROR;
      }
    }
  }

  size_t iteration = 0;
  for (auto & sub : discovered_subscribers_) {
    if (sub.uses_cpu_group) {
      continue;
    }

    iteration++;
    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Processing message for subscriber endpoint %zu/%zu with key='%s'",
      iteration, discovered_subscribers_.size(), sub.endpoint_key.c_str());

    auto endpoint_it = endpoints_.find(sub.endpoint_key);
    if (endpoint_it == endpoints_.end() || !endpoint_it->second) {
      continue;
    }
    auto endpoint = endpoint_it->second;

    // The generated typesupport estimate includes the CDR encapsulation and
    // the bounded descriptor size used by endpoint-aware Buffer serialization.
    const size_t max_data_length = type_support_->get_estimated_serialized_size(
      ros_message, type_support_impl_);

    rcutils_allocator_t * allocator = &rmw_node_->context->options.allocator;
    void * data = allocator->allocate(max_data_length, allocator->state);
    if (!data) {
      RMW_SET_ERROR_MSG("failed to allocate serialization buffer");
      return RMW_RET_BAD_ALLOC;
    }

    auto always_free_data = rcpputils::make_scope_exit(
      [data, allocator]() {
        allocator->deallocate(data, allocator->state);
      });

    uint8_t * msg_bytes = static_cast<uint8_t *>(data);
    eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char *>(msg_bytes), max_data_length);
    rmw_zenoh_cpp::Cdr ser(fastbuffer);

    bool ok = type_support_->serialize_ros_message_with_endpoint(
      ros_message, ser.get_cdr(), type_support_impl_, sub.endpoint_info.info,
      backend_ctx->serialization_context);
    if (!ok) {
      RMW_SET_ERROR_MSG("could not serialize ROS message with endpoint awareness");
      return RMW_RET_ERROR;
    }

    size_t data_length = ser.get_serialized_data_length();

    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Serialization complete, actual size: %zu bytes (allocated: %zu, usage: %.1f%%)",
      data_length, max_data_length, (data_length * 100.0) / max_data_length);

    // Sanity check: ensure we didn't overflow
    if (data_length > max_data_length) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "[Publisher] CRITICAL: Serialized size %zu exceeds allocated buffer %zu!",
        data_length, max_data_length);
      return RMW_RET_ERROR;
    }

    // Publish to this endpoint
    // Use deleter to manage memory since Zenoh takes ownership
    auto deleter = [data, allocator](uint8_t *) {
        allocator->deallocate(data, allocator->state);
      };
    auto payload = zenoh::Bytes(msg_bytes, data_length, deleter);
    always_free_data.cancel();  // Zenoh now owns the memory

    // Create attachment AFTER serialization
    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Creating message attachment data");

    int64_t source_timestamp = rmw_zenoh_cpp::get_system_time_in_ns();
    auto gid = entity_->copy_gid();

    auto attachment_data = rmw_zenoh_cpp::AttachmentData(
      sequence_number_++, source_timestamp, gid);

    auto attachment_bytes = attachment_data.serialize_to_zbytes();

    zenoh::ext::AdvancedPublisher::PutOptions options =
      zenoh::ext::AdvancedPublisher::PutOptions::create_default();
    // Set attachment directly without std::make_optional (same as non-buffer-aware path)
    options.put_options.attachment = std::move(attachment_bytes);

    zenoh::ZResult result;
    if (endpoint->pub.has_value()) {
      RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp",
        "[Publisher] Calling Zenoh put");

      endpoint->pub.value().put(std::move(payload), std::move(options), &result);

      if (result != Z_OK) {
        RMW_ZENOH_ROSIDL_BUFFER_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Failed to publish to endpoint '%s'", sub.endpoint_key.c_str());
        ret = RMW_RET_ERROR;
      }
    }
  }

  return ret;
}

///=============================================================================
rmw_ret_t PublisherData::publish(
  const void * ros_message,
  ShmContext * shm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    RMW_SET_ERROR_MSG("Unable to publish as the publisher has been shutdown.");
    return RMW_RET_ERROR;
  }

  // Buffer-aware publishing strategy (mirrors rmw_fastrtps):
  //   - If ALL matched subscribers are buffer-aware, publish only to
  //     per-subscriber endpoint keys (accelerated path).
  //   - If ANY legacy (non-buffer-aware) subscriber exists, skip buffer
  //     channels entirely and fall through to the standard base-key publish.
  //     Buffer-aware subscribers also have a base-key subscription so they
  //     will still receive the message via standard deserialization.
  if (is_buffer_aware_) {
    size_t total_matched = 0;
    if (graph_cache_) {
      graph_cache_->publisher_count_matched_subscriptions(
        entity_->topic_info().value(), &total_matched);
    }
    if (total_matched <= discovered_subscribers_.size()) {
      return publish_buffer_aware(ros_message, shm);
    }
    // Legacy subscribers present -- fall through to standard serialization
    // on the base key so every subscriber (buffer-aware and legacy) receives
    // the message.
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
    if (auto shm_provider = shm->get_shm_provider(*sess_)) {
      RMW_ZENOH_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "SHM is enabled.");

      auto alloc_result = shm_provider.value().shm_provider().alloc_gc_defrag(max_data_length);

      if (std::holds_alternative<zenoh::ZShmMut>(alloc_result)) {
        auto && buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
        msg_bytes = reinterpret_cast<uint8_t *>(buf.data());
        shm_buf = std::make_optional(std::move(buf));
      } else {
        // Print a warning and revert to regular allocation
        RMW_ZENOH_LOG_DEBUG_NAMED(
          "rmw_zenoh_cpp", "Failed to allocate a SHM buffer, fallback to non-SHM");
      }
    } else {
      // Print a warning and revert to regular allocation
      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "SHM provider is not yet available, fallback to non-SHM");
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
        buffer = pool_buf](uint8_t *) {
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
  base_endpoint_->pub.value().put(std::move(payload), std::move(opts), &result);
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
  ShmContext * shm)
{
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  rmw_zenoh_cpp::Cdr ser(buffer);
  if (!ser.get_cdr().jump(serialized_message->buffer_length)) {
    RMW_SET_ERROR_MSG("cannot correctly set serialized buffer");
    return RMW_RET_ERROR;
  }

  // Optional shared memory buffer
  std::optional<zenoh::ZShmMut> shm_buf = std::nullopt;

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
    if (auto shm_provider = shm->get_shm_provider(*sess_)) {
      RMW_ZENOH_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "SHM is enabled.");

      auto alloc_result = shm_provider.value().shm_provider().alloc_gc_defrag(data_length);

      if (std::holds_alternative<zenoh::ZShmMut>(alloc_result)) {
        auto && buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
        shm_buf = std::make_optional(std::move(buf));
      } else {
        // Print a warning and revert to regular allocation
        RMW_ZENOH_LOG_DEBUG_NAMED(
          "rmw_zenoh_cpp", "Failed to allocate a SHM buffer, fallback to non-SHM");
      }
    } else {
      // Print a warning and revert to regular allocation
      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "SHM provider is not yet available, fallback to non-SHM");
    }
  }

  if (shm_buf.has_value()) {
    auto msg_bytes = reinterpret_cast<char *>(shm_buf.value().data());
    memcpy(msg_bytes, serialized_message->buffer, data_length);
    zenoh::Bytes payload(std::move(*shm_buf));

    TRACETOOLS_TRACEPOINT(
      rmw_publish, static_cast<const void *>(rmw_publisher_), serialized_message,
      source_timestamp);

    base_endpoint_->pub.value().put(std::move(payload), std::move(opts), &result);
  } else {
    std::vector<uint8_t> raw_image(
      serialized_message->buffer,
      serialized_message->buffer + data_length);
    zenoh::Bytes payload(std::move(raw_image));

    TRACETOOLS_TRACEPOINT(
      rmw_publish, static_cast<const void *>(rmw_publisher_), serialized_message,
        source_timestamp);

    base_endpoint_->pub.value().put(std::move(payload), std::move(opts), &result);
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
void PublisherData::on_subscriber_discovered(const liveliness::Entity & entity)
{
  if (entity.type() != liveliness::EntityType::Subscription) {
    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Publisher] Ignoring discovered entity type=%s node='%s' ns='%s'",
      entity_type_to_string(entity.type()),
      entity.node_name().c_str(),
      entity.node_namespace().c_str());
    return;
  }

  auto topic_info_opt = entity.topic_info();
  if (!topic_info_opt.has_value()) {
    RMW_ZENOH_ROSIDL_BUFFER_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Discovered subscriber without topic info on Buffer topic");
    return;
  }

  std::unordered_map<std::string, std::string> sub_backend_metadata;
  if (topic_info_opt->backend_metadata_.has_value()) {
    sub_backend_metadata = topic_info_opt->backend_metadata_.value();
  }
  const bool use_cpu_group = is_cpu_only_backend_metadata(sub_backend_metadata);

  auto gid = entity_gid_to_rmw_gid(entity, rmw_zenoh_identifier);
  const auto entity_gid_array = entity.copy_gid();
  RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[Publisher] Discovered subscriber entity keyexpr='%s', "
    "topic='%s', entity.type='%s', node='%s', ns='%s', "
    "zid='%s', gid='%s', entity_gid='%s'",
    entity.liveliness_keyexpr().c_str(),
    topic_info_opt->name_.c_str(),
    entity_type_to_string(entity.type()),
    entity.node_name().c_str(),
    entity.node_namespace().c_str(),
    entity.zid().c_str(),
    gid_to_hex(gid).c_str(),
    gid_array_to_hex(entity_gid_array).c_str());

  auto sub_endpoint_info = build_endpoint_info_from_entity(entity, RMW_ENDPOINT_SUBSCRIPTION);

  // Phase 1: collect state under lock, check duplicates, mark pending
  std::string full_key;
  std::vector<rmw_topic_endpoint_info_t> existing_endpoints;
  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> backend_endpoint_groups;
  bool need_create_endpoint = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_buffer_aware_ || is_shutdown_) {
      return;
    }

    for (const auto & existing : discovered_subscribers_) {
      if (memcmp(existing.gid.data, gid.data, RMW_GID_STORAGE_SIZE) == 0) {
        return;
      }
    }

    existing_endpoints.reserve(1 + discovered_subscribers_.size());
    existing_endpoints.push_back(local_endpoint_info_.info);
    for (const auto & existing : discovered_subscribers_) {
      existing_endpoints.push_back(existing.endpoint_info.info);
    }

    for (const auto & existing : discovered_subscribers_) {
      backend_endpoint_groups.insert(existing.backend_groups.begin(),
        existing.backend_groups.end());
    }

    full_key = use_cpu_group ?
      make_cpu_group_key(entity_->topic_info()->topic_keyexpr_) :
      make_accelerated_key(entity_->topic_info()->topic_keyexpr_, gid);

    need_create_endpoint = (endpoints_.find(full_key) == endpoints_.end());
    if (need_create_endpoint) {
      if (!pending_endpoints_.insert(full_key).second) {
        return;
      }
    }
  }

  // Phase 2: external operations without lock
  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> backend_groups;
  {
    if (!use_cpu_group) {
      rmw_context_impl_t * ctx_impl = static_cast<rmw_context_impl_t *>(rmw_node_->context->impl);
      auto * backend_ctx = ctx_impl->buffer_backend_context();
      if (backend_ctx) {
        (void)rosidl_buffer_backend_registry::notify_endpoint_discovered(
          backend_ctx->backend_instances,
          sub_endpoint_info.info, existing_endpoints, backend_endpoint_groups,
          sub_backend_metadata);
      }
    }
  }

  std::shared_ptr<PublisherEndpoint> new_endpoint;
  if (need_create_endpoint) {
    new_endpoint = create_publisher_endpoint(full_key);
    if (!new_endpoint) {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_endpoints_.erase(full_key);
      return;
    }
  }

  // Phase 3: store results under lock
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (new_endpoint) {
      endpoints_[full_key] = new_endpoint;
      pending_endpoints_.erase(full_key);
    }

    SubscriberInfo sub_info;
    sub_info.gid = gid;
    sub_info.endpoint_key = full_key;
    sub_info.endpoint_info = std::move(sub_endpoint_info);
    sub_info.uses_cpu_group = use_cpu_group;
    sub_info.backend_metadata = sub_backend_metadata;
    sub_info.backend_groups = std::move(backend_groups);
    discovered_subscribers_.push_back(std::move(sub_info));

    if (endpoints_.count(full_key)) {
      endpoints_[full_key]->target_subscribers.push_back(gid);
    }
  }
}

///=============================================================================
std::shared_ptr<PublisherData::PublisherEndpoint> PublisherData::get_or_create_endpoint(
  const std::string & full_key)
{
  auto it = endpoints_.find(full_key);
  if (it != endpoints_.end()) {
    return it->second;
  }

  auto endpoint = create_publisher_endpoint(full_key);
  if (endpoint) {
    endpoints_[full_key] = endpoint;
  }
  return endpoint;
}

///=============================================================================
std::shared_ptr<PublisherData::PublisherEndpoint> PublisherData::create_publisher_endpoint(
  const std::string & full_key, bool buffer_aware)
{
  auto endpoint = std::make_shared<PublisherEndpoint>();
  endpoint->key = full_key;

  zenoh::KeyExpr pub_ke(full_key);

  auto qos_profile = entity_->topic_info()->qos_;

  using AdvancedPublisherOptions = zenoh::ext::SessionExt::AdvancedPublisherOptions;
  using SampleMissDetectionOptions = AdvancedPublisherOptions::SampleMissDetectionOptions;
  auto adv_pub_opts = AdvancedPublisherOptions::create_default();

  if (qos_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    // Allow this publisher to be detected through liveliness.
    adv_pub_opts.publisher_detection = true;
    adv_pub_opts.cache = AdvancedPublisherOptions::CacheOptions::create_default();
    adv_pub_opts.cache->max_samples = qos_profile.depth;
    // Sample miss detection (heartbeat) is only used on the base
    // publisher. If RELIABLE + TRANSIENT_LOCAL it lets subscribers detect missed
    // samples and retrieve them from the Publisher cache; HeartbeatSporadic keeps
    // background traffic low. Per-subscriber buffer-aware endpoints are
    // point-to-point and do not need it.
    if (!buffer_aware && qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      adv_pub_opts.sample_miss_detection = SampleMissDetectionOptions{};
      adv_pub_opts.sample_miss_detection->heartbeat =
        SampleMissDetectionOptions::HeartbeatSporadic{
        SAMPLE_MISS_DETECTION_HEARTBEAT_PERIOD};
    }
  }

  auto pub_opts = zenoh::Session::PublisherOptions::create_default();
  pub_opts.congestion_control = Z_CONGESTION_CONTROL_DROP;
  if (qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
    pub_opts.reliability = Z_RELIABILITY_RELIABLE;
    if (qos_profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
      pub_opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
    }
  } else {
    pub_opts.reliability = Z_RELIABILITY_BEST_EFFORT;
  }
  adv_pub_opts.publisher_options = pub_opts;

  zenoh::ZResult result;
  auto pub = sess_->ext().declare_advanced_publisher(
      pub_ke, std::move(adv_pub_opts), &result);

  if (result != Z_OK) {
    RMW_ZENOH_ROSIDL_BUFFER_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Failed to create publisher endpoint for key: %s", full_key.c_str());
    return nullptr;
  }

  endpoint->pub = std::optional<zenoh::ext::AdvancedPublisher>(std::move(pub));
  return endpoint;
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
  std::unordered_map<std::string, std::shared_ptr<PublisherEndpoint>> endpoints_to_destroy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_) {
      return RMW_RET_OK;
    }
    is_shutdown_ = true;
    base_endpoint_ = nullptr;
    endpoints_to_destroy = std::move(endpoints_);
    endpoints_.clear();
  }

  zenoh::ZResult result;
  // Undeclare every publisher endpoint: the base publisher plus any
  // buffer-aware endpoints.
  for (auto & [key, endpoint] : endpoints_to_destroy) {
    if (endpoint->pub.has_value()) {
      std::move(endpoint->pub.value()).undeclare(&result);
      if (result != Z_OK) {
        RMW_ZENOH_ROSIDL_BUFFER_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Failed to undeclare endpoint with key '%s'", key.c_str());
      }
    }
  }

  std::move(token_).value().undeclare(&result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to undeclare the liveliness token for topic '%s'",
      entity_->topic_info().value().name_.c_str());
    return RMW_RET_ERROR;
  }

  sess_.reset();
  return RMW_RET_OK;
}

///=============================================================================
bool PublisherData::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}
}  // namespace rmw_zenoh_cpp
