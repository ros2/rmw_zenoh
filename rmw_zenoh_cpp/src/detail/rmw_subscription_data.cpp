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

#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <variant>

#include "attachment_helpers.hpp"
#include "cdr.hpp"

#include "rosidl_buffer_backend_registry/backend_utils.hpp"
#include "buffer_backend_context.hpp"
#include "buffer_endpoint_helpers.hpp"
#include "identifier.hpp"
#include "rmw_context_impl_s.hpp"
#include "message_type_support.hpp"
#include "logging_macros.hpp"
#include "qos.hpp"
#include "liveliness_utils.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rosidl_runtime_c/type_hash.h"

#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

namespace rmw_zenoh_cpp
{

using buffer_endpoint_helpers::entity_type_to_string;
using buffer_endpoint_helpers::gid_array_to_hex;
using buffer_endpoint_helpers::gid_to_hex;
using buffer_endpoint_helpers::is_cpu_only_backend_types;
using buffer_endpoint_helpers::make_accelerated_key;
using buffer_endpoint_helpers::make_cpu_group_key;
///=============================================================================
SubscriptionData::Message::Message(
  const zenoh::Bytes & p,
  uint64_t recv_ts,
  AttachmentData && attachment_,
  std::optional<EndpointInfoStorage> endpoint_info_)
: payload(p), recv_timestamp(recv_ts), attachment(std::move(attachment_)),
  endpoint_info(std::move(endpoint_info_))
{
}

///=============================================================================
std::shared_ptr<SubscriptionData> SubscriptionData::make(
  std::shared_ptr<zenoh::Session> session,
  std::shared_ptr<GraphCache> graph_cache,
  const rmw_node_t * const node,
  liveliness::NodeInfo node_info,
  std::size_t node_id,
  std::size_t subscription_id,
  const std::string & topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile,
  const rmw_subscription_options_t & sub_options)
{
  rmw_qos_profile_t adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = QoS::get().best_available_qos(
    node, topic_name.c_str(), &adapted_qos_profile, rmw_get_publishers_info_by_topic);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  const rosidl_type_hash_t * type_hash = type_support->get_type_hash_func(type_support);
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);
  auto message_type_support = std::make_unique<MessageTypeSupport>(callbacks);

  const bool has_buffer_fields = callbacks->has_buffer_fields;
  // A transient-local subscription can only receive history through its base
  // advanced subscriber, so its custom buffer endpoints would never provide it.
  const bool use_buffer_endpoints = has_buffer_fields &&
    adapted_qos_profile.durability != RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  RMW_ZENOH_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[SubscriptionData::make] Topic: %s, has_buffer_fields: %d, "
    "use_buffer_endpoints: %d",
    topic_name.c_str(), has_buffer_fields, use_buffer_endpoints);

  // Query and filter installed backends based on acceptable_buffer_backends option
  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(node->context->impl);
  std::optional<std::unordered_map<std::string, std::string>> backend_types = std::nullopt;
  std::vector<std::string> my_backend_types;
  if (use_buffer_endpoints) {
    auto * backend_ctx = context_impl->buffer_backend_context();
    std::vector<std::string> all_installed;
    std::unordered_map<std::string, std::string> all_backend_metadata;
    if (backend_ctx) {
      for (const auto & [name, _] : backend_ctx->backend_instances) {
        all_installed.push_back(name);
      }
      all_backend_metadata = rosidl_buffer_backend_registry::get_all_backend_metadata(
        backend_ctx->backend_instances);
    }

    const char * requested = sub_options.acceptable_buffer_backends;

    // Parse comma-separated list (if provided and non-empty)
    std::vector<std::string> requested_list;
    if (requested != nullptr && strlen(requested) > 0) {
      std::istringstream stream(requested);
      std::string token;
      while (std::getline(stream, token, ',')) {
        size_t start = token.find_first_not_of(" \t");
        size_t end = token.find_last_not_of(" \t");
        if (start != std::string::npos) {
          requested_list.push_back(token.substr(start, end - start + 1));
        }
      }
    }

    // "any": accept all installed backends
    bool use_all = false;
    for (const auto & name : requested_list) {
      if (name == "any") {
        use_all = true;
        break;
      }
    }

    // NULL, empty, or only "cpu" entries: CPU-only (backward compat default)
    bool cpu_only = !use_all && (requested_list.empty() ||
      std::all_of(requested_list.begin(), requested_list.end(),
      [](const std::string & n) {return n == "cpu";}));

    if (use_all) {
      my_backend_types = all_installed;
      backend_types = all_backend_metadata;

      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp",
        "Creating Buffer-aware subscription for topic %s with %zu backends (all installed)",
        topic_name.c_str(), my_backend_types.size());
    } else if (cpu_only) {
      // CPU-only: advertise "cpu" as the only supported backend so the
      // subscription stays on the buffer-aware per-endpoint route and
      // passes the backends_compatible check with CPU-only publishers.
      my_backend_types.push_back("cpu");
      backend_types = std::unordered_map<std::string, std::string>{{"cpu", ""}};

      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp",
        "Creating CPU-only Buffer subscription for topic %s "
        "(acceptable_buffer_backends='%s')",
        topic_name.c_str(), requested ? requested : "(null)");
    } else {
      // Validate: every requested non-CPU backend must be installed
      for (const auto & name : requested_list) {
        if (name == "cpu") {
          continue;
        }
        if (std::find(all_installed.begin(), all_installed.end(), name) ==
          all_installed.end())
        {
          std::string available_str;
          for (size_t i = 0; i < all_installed.size(); ++i) {
            if (i > 0) {available_str += ", ";}
            available_str += all_installed[i];
          }
          RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
            "Buffer backend '%s' specified in acceptable_buffer_backends "
            "is not installed. Available backends: %s",
            name.c_str(), available_str.c_str());
          return nullptr;
        }
      }

      // Filter: only include requested backends and collect their backend metadata
      std::unordered_map<std::string, std::string> filtered_backend_metadata;
      for (const auto & name : requested_list) {
        if (name == "cpu") {
          continue;
        }
        my_backend_types.push_back(name);
        auto meta_it = all_backend_metadata.find(name);
        if (meta_it != all_backend_metadata.end()) {
          filtered_backend_metadata[name] = meta_it->second;
        }
      }
      backend_types = filtered_backend_metadata;

      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp",
        "Creating subscription for topic %s with %zu filtered backends "
        "(from acceptable_buffer_backends='%s')",
        topic_name.c_str(), my_backend_types.size(), requested);
    }
  }

  // Convert the type hash to a string so that it can be included in the keyexpr.
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(
    type_hash,
    *allocator,
    &type_hash_c_str);
  if (RCUTILS_RET_BAD_ALLOC == stringify_ret) {
    // rosidl_stringify_type_hash already set the error
    return nullptr;
  }
  auto free_type_hash_c_str = rcpputils::make_scope_exit(
    [&allocator, &type_hash_c_str]() {
      allocator->deallocate(type_hash_c_str, allocator->state);
    });

  // Everything above succeeded and is setup properly. Now declare a subscriber
  // with Zenoh; after this, callbacks may come in at any time.
  std::size_t domain_id = node_info.domain_id_;
  auto entity = liveliness::Entity::make(
    session->get_zid(),
    std::to_string(node_id),
    std::to_string(subscription_id),
    liveliness::EntityType::Subscription,
    std::move(node_info),
    liveliness::TopicInfo{
      std::move(domain_id),
      topic_name,
      message_type_support->get_name(),
      type_hash_c_str,
      adapted_qos_profile,
      backend_types}  // Include backends only when buffer endpoints are enabled.
  );
  if (entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the subscription %s.",
      topic_name.c_str());
    return nullptr;
  }

  auto sub_data = std::shared_ptr<SubscriptionData>(
    new SubscriptionData{
      node,
      graph_cache,
      std::move(entity),
      std::move(session),
      type_support->data,
      std::move(message_type_support),
      sub_options,
      use_buffer_endpoints,
      my_backend_types
    });

  if (use_buffer_endpoints) {
    sub_data->local_endpoint_info_ =
      build_endpoint_info_from_entity(*sub_data->entity_, RMW_ENDPOINT_SUBSCRIPTION);

    auto * backend_ctx = context_impl->buffer_backend_context();
    if (backend_ctx) {
      rosidl_buffer_backend_registry::notify_endpoint_created(
        backend_ctx->backend_instances, sub_data->local_endpoint_info_.info);
    }
  }

  if (!sub_data->init()) {
    // init() already set the error
    return nullptr;
  }

  // Register discovery callback for Buffer-aware subscribers
  if (use_buffer_endpoints && graph_cache != nullptr) {
    std::weak_ptr<SubscriptionData> weak_sub_data = sub_data;
    if (!is_cpu_only_backend_types(my_backend_types)) {
      graph_cache->register_publisher_discovery_callback(
        topic_name,
        sub_data->gid_hash(),
        [weak_sub_data](const liveliness::Entity & entity) {
          if (auto sd = weak_sub_data.lock()) {
            sd->on_publisher_discovered(entity);
          }
        });
    }

    // Manually add this local subscriber to the graph cache so local publishers can discover it
    // Liveliness events from the same session don't trigger graph updates automatically
    graph_cache->parse_put(sub_data->entity_->liveliness_keyexpr(), false);
    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] Manually added local subscriber to graph cache for discovery");
  }

  return sub_data;
}

///=============================================================================
SubscriptionData::SubscriptionData(
  const rmw_node_t * rmw_node,
  std::shared_ptr<GraphCache> graph_cache,
  std::shared_ptr<liveliness::Entity> entity,
  std::shared_ptr<zenoh::Session> session,
  const void * type_support_impl,
  std::unique_ptr<MessageTypeSupport> type_support,
  rmw_subscription_options_t sub_options,
  bool is_buffer_aware,
  std::vector<std::string> my_backend_types)
: rmw_node_(rmw_node),
  graph_cache_(std::move(graph_cache)),
  entity_(std::move(entity)),
  sess_(std::move(session)),
  type_support_impl_(type_support_impl),
  type_support_(std::move(type_support)),
  sub_options_(std::move(sub_options)),
  last_known_published_msg_({}),
  reception_sn_(0),
  wait_set_data_(nullptr),
  is_shutdown_(false),
  initialized_(false),
  is_buffer_aware_(is_buffer_aware),
  my_backend_types_(std::move(my_backend_types))
{
  events_mgr_ = std::make_shared<EventsManager>();
}

///=============================================================================
// We have to use an "init" function here, rather than do this in the constructor, because we use
// enable_shared_from_this, which is not available in constructors.
bool SubscriptionData::init()
{
  zenoh::ZResult result;
  zenoh::KeyExpr sub_ke(entity_->topic_info()->topic_keyexpr_, true, &result);
  if (result != Z_OK) {
    RMW_SET_ERROR_MSG("unable to create zenoh keyexpr.");
    return false;
  }

  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(rmw_node_->context->impl);

  sess_ = context_impl->session();

  using AdvancedSubscriberOptions = zenoh::ext::SessionExt::AdvancedSubscriberOptions;
  using RecoveryOptions = AdvancedSubscriberOptions::RecoveryOptions;
  auto adv_sub_opts = AdvancedSubscriberOptions::create_default();

  // By default, this subscription will receive publications from publishers within and outside of
  // the same Zenoh session as this subscription.
  // If ignore_local_publications is true, we restrict this subscription to only receive samples
  // from publishers in remote sessions.
  if (sub_options_.ignore_local_publications) {
    adv_sub_opts.subscriber_options.allowed_origin = ZC_LOCALITY_REMOTE;
  }

  // Instantiate the subscription with suitable options depending on the
  // adapted_qos_profile.
  if (entity_->topic_info()->qos_.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    // Allow this subscriber to be detected through liveliness.
    adv_sub_opts.subscriber_detection = true;
    adv_sub_opts.query_timeout_ms = std::numeric_limits<uint64_t>::max();
    // History can only be retransmitted by Publishers that enable caching.
    adv_sub_opts.history = AdvancedSubscriberOptions::HistoryOptions::create_default();
    // Enable detection of late joiner publishers and query for their historical data.
    adv_sub_opts.history->detect_late_publishers = true;
    adv_sub_opts.history->max_samples = entity_->topic_info()->qos_.depth;
    if (entity_->topic_info()->qos_.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      // Activate recovery of lost samples.
      // This requires the Publisher to have sample_miss_detection configured,
      // which is the case for a RELIABLE + TRANSIENT_LOCAL Publisher.
      adv_sub_opts.recovery = AdvancedSubscriberOptions::RecoveryOptions{};
      adv_sub_opts.recovery->last_sample_miss_detection = RecoveryOptions::Heartbeat{};
    }
  }

  // Always create the base key subscription.  For non-buffer-aware messages
  // this is the only subscription.  For buffer-aware messages this acts as a
  // fallback so the subscriber can still receive standard-serialized data from
  // legacy publishers or when the publisher detects mixed (legacy + buffer)
  // subscribers and falls back to base-key-only publishing.
  // Additional buffer-aware subscriptions are either created dynamically in
  // on_publisher_discovered() for non-CPU backends or bound to the shared
  // CPU channel below for CPU-only subscriptions.
  {
    std::weak_ptr<SubscriptionData> data_wp = shared_from_this();
    auto on_sample = [data_wp](const zenoh::Sample & sample) {
        auto sub_data = data_wp.lock();
        if (sub_data == nullptr) {
          RMW_ZENOH_LOG_ERROR_NAMED(
            "rmw_zenoh_cpp",
            "SubscriberCallback triggered over %s.",
            std::string(sample.get_keyexpr().as_string_view()).c_str()
          );
          return;
        }
        auto attachment = sample.get_attachment();
        if (!attachment.has_value()) {
          RMW_ZENOH_LOG_ERROR_NAMED(
            "rmw_zenoh_cpp",
            "Unable to obtain attachment for topic '%s'",
            std::string(sample.get_keyexpr().as_string_view()).c_str())
          return;
        }
        auto attachment_value = attachment.value();

        AttachmentData attachment_data(attachment_value);
        sub_data->add_new_message(
          std::make_unique<SubscriptionData::Message>(
            sample.get_payload(),
            get_system_time_in_ns(),
            std::move(attachment_data)),
          std::string(sample.get_keyexpr().as_string_view()));
      };
    sub_ = context_impl->session()->ext().declare_advanced_subscriber(
      sub_ke,
      std::move(on_sample),
      zenoh::closures::none,
      std::move(adv_sub_opts),
      &result);
    if (result != Z_OK) {
      RMW_SET_ERROR_MSG("unable to create zenoh subscription");
      return false;
    }
  }

  if (is_buffer_aware_) {
    if (is_cpu_only_backend_types(my_backend_types_)) {
      auto cpu_endpoint = create_subscription_endpoint(
        make_cpu_group_key(entity_->topic_info()->topic_keyexpr_),
        std::nullopt,
        false);
      if (!cpu_endpoint) {
        RMW_SET_ERROR_MSG("unable to create zenoh CPU subscription");
        return false;
      }
      sub_endpoints_[cpu_endpoint->key] = cpu_endpoint;
    } else {
      // Eagerly create a single shared accelerated channel keyed by this
      // subscriber's GID. All buffer-aware publishers that match this
      // subscriber publish to this key, so one zenoh subscriber serves all
      // of them.  Mirrors rmw_fastrtps_cpp's "single shared accelerated
      // DataReader per subscriber" design.
      rmw_gid_t local_gid = rmw_zenoh_cpp::entity_gid_to_rmw_gid(
        *entity_, rmw_zenoh_cpp::rmw_zenoh_identifier);
      const std::string accel_key = make_accelerated_key(
        entity_->topic_info()->topic_keyexpr_, gid_to_hex(local_gid));
      auto accel_endpoint = create_subscription_endpoint(
        accel_key, std::nullopt, true);
      if (!accel_endpoint) {
        RMW_SET_ERROR_MSG("unable to create zenoh accelerated subscription");
        return false;
      }
      sub_endpoints_[accel_endpoint->key] = accel_endpoint;
    }
  }

  // Publish to the graph that a new subscription is in town.
  std::string liveliness_keyexpr = entity_->liveliness_keyexpr();
  token_ = context_impl->session()->liveliness_declare_token(
    zenoh::KeyExpr(liveliness_keyexpr),
    zenoh::Session::LivelinessDeclarationOptions::create_default(),
    &result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the subscription.");
    return false;
  }

  initialized_ = true;

  if (is_buffer_aware_) {
    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] Initialized buffer-aware subscription, "
      "base key: '%s'",
      entity_->topic_info()->topic_keyexpr_.c_str());
  } else {
    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] Initialized simple subscription on key: '%s'",
      entity_->topic_info()->topic_keyexpr_.c_str());
  }

  return true;
}

///=============================================================================
std::size_t SubscriptionData::gid_hash() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->gid_hash();
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
  // The z_check function is now internal in zenoh-1.0.0 so we assume
  // the liveliness token is still initialized as long as this entity has
  // not been shutdown.
  return !is_shutdown_;
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
void SubscriptionData::on_publisher_discovered(const liveliness::Entity & entity)
{
  RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[Subscription] on_publisher_discovered callback triggered!");

  if (entity.type() != liveliness::EntityType::Publisher) {
    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] Ignoring discovered entity type=%s node='%s' ns='%s'",
      entity_type_to_string(entity.type()),
      entity.node_name().c_str(),
      entity.node_namespace().c_str());
    return;
  }

  const auto & topic_info = entity.topic_info();
  if (!topic_info.has_value()) {
    RMW_ZENOH_ROSIDL_BUFFER_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Discovered publisher without topic info on Buffer topic");
    return;
  }

  std::unordered_map<std::string, std::string> pub_backend_metadata;
  std::vector<std::string> pub_backends;
  if (topic_info->backend_metadata_.has_value() && !topic_info->backend_metadata_->empty()) {
    pub_backend_metadata = topic_info->backend_metadata_.value();
    pub_backends.reserve(pub_backend_metadata.size() + 1);
    for (const auto & pair : pub_backend_metadata) {
      pub_backends.push_back(pair.first);
    }
  }
  // CPU serialization is always implicitly supported by all buffer-aware publishers
  pub_backends.push_back("cpu");

  if (!rosidl_buffer_backend_registry::backends_compatible(
      my_backend_types_, pub_backends))
  {
    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Discovered publisher with incompatible backends, skipping");
    return;
  }

  rmw_gid_t pub_gid = rmw_zenoh_cpp::entity_gid_to_rmw_gid(entity,
      rmw_zenoh_cpp::rmw_zenoh_identifier);

  auto pub_endpoint_info = build_endpoint_info_from_entity(entity, RMW_ENDPOINT_PUBLISHER);
  if (is_cpu_only_backend_types(my_backend_types_)) {
    return;
  }

  // Phase 1: collect state under lock, check duplicates.
  // A single shared accelerated endpoint was already created in init();
  // here we only need to register the discovered publisher's metadata so
  // take_one_message / the accelerated channel callback can resolve the
  // sending publisher's endpoint_info by GID lookup.
  std::vector<rmw_topic_endpoint_info_t> existing_endpoints;
  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> backend_endpoint_groups;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_buffer_aware_ || is_shutdown_) {
      return;
    }

    for (const auto & existing : discovered_publishers_) {
      if (memcmp(existing.gid.data, pub_gid.data, RMW_GID_STORAGE_SIZE) == 0) {
        return;
      }
    }

    existing_endpoints.reserve(1 + discovered_publishers_.size());
    existing_endpoints.push_back(local_endpoint_info_.info);
    for (const auto & existing : discovered_publishers_) {
      existing_endpoints.push_back(existing.endpoint_info.info);
    }

    for (const auto & existing : discovered_publishers_) {
      backend_endpoint_groups.insert(existing.backend_groups.begin(),
        existing.backend_groups.end());
    }
  }

  RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[Subscription] Discovered publisher entity keyexpr='%s', "
    "topic='%s', entity.type='%s', node='%s', ns='%s', "
    "zid='%s', gid='%s', entity_gid='%s'",
    entity.liveliness_keyexpr().c_str(),
    entity.topic_info().has_value() ? entity.topic_info()->name_.c_str() : "",
    entity_type_to_string(entity.type()),
    entity.node_name().c_str(),
    entity.node_namespace().c_str(),
    entity.zid().c_str(),
    gid_to_hex(pub_gid).c_str(),
    gid_array_to_hex(entity.copy_gid()).c_str());

  // Phase 2: notify backend of the discovered publisher without holding the lock.
  std::unordered_map<std::string, std::vector<std::set<uint32_t>>> backend_groups;
  {
    rmw_context_impl_t * ctx_impl = static_cast<rmw_context_impl_t *>(rmw_node_->context->impl);
    auto * backend_ctx = ctx_impl->buffer_backend_context();
    if (backend_ctx) {
      (void)rosidl_buffer_backend_registry::notify_endpoint_discovered(
        backend_ctx->backend_instances,
        pub_endpoint_info.info, existing_endpoints, backend_endpoint_groups,
        pub_backend_metadata);
    }
  }

  // Phase 3: record the publisher's metadata under the lock.  The shared
  // accelerated endpoint (created in init()) already receives messages from
  // any buffer-aware publisher that matches this subscriber; we do not
  // need to create a per-publisher zenoh subscription here.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PublisherInfo pub_info;
    pub_info.gid = pub_gid;
    pub_info.endpoint_key.clear();
    pub_info.endpoint_info = std::move(pub_endpoint_info);
    pub_info.backend_metadata = pub_backend_metadata;
    pub_info.backend_groups = std::move(backend_groups);
    discovered_publishers_.push_back(std::move(pub_info));
  }
}

///=============================================================================
std::optional<EndpointInfoStorage> SubscriptionData::lookup_publisher_endpoint_info(
  const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & publisher_gid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & pub_info : discovered_publishers_) {
    if (std::memcmp(pub_info.gid.data, publisher_gid.data(), RMW_GID_STORAGE_SIZE) == 0) {
      return pub_info.endpoint_info;
    }
  }
  return std::nullopt;
}

///=============================================================================
void SubscriptionData::create_subscription_for_key(
  const std::string & key,
  std::optional<EndpointInfoStorage> publisher_info)
{
  auto endpoint = create_subscription_endpoint(key, publisher_info, false);
  if (endpoint) {
    sub_endpoints_[key] = endpoint;
  }
}

///=============================================================================
std::shared_ptr<SubscriptionData::SubscriptionEndpoint>
SubscriptionData::create_subscription_endpoint(
  const std::string & key,
  std::optional<EndpointInfoStorage> publisher_info,
  bool is_accelerated)
{
  zenoh::ZResult result;
  zenoh::KeyExpr sub_ke(key, true, &result);
  if (result != Z_OK) {
    RMW_ZENOH_ROSIDL_BUFFER_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create zenoh keyexpr for key: %s", key.c_str());
    return nullptr;
  }

  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(rmw_node_->context->impl);

  using AdvancedSubscriberOptions = zenoh::ext::SessionExt::AdvancedSubscriberOptions;
  using RecoveryOptions = AdvancedSubscriberOptions::RecoveryOptions;
  auto adv_sub_opts = AdvancedSubscriberOptions::create_default();

  if (sub_options_.ignore_local_publications) {
    adv_sub_opts.subscriber_options.allowed_origin = ZC_LOCALITY_REMOTE;
  }

  if (entity_->topic_info()->qos_.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    adv_sub_opts.subscriber_detection = true;
    adv_sub_opts.query_timeout_ms = std::numeric_limits<uint64_t>::max();
    adv_sub_opts.history = AdvancedSubscriberOptions::HistoryOptions::create_default();
    adv_sub_opts.history->detect_late_publishers = true;
    adv_sub_opts.history->max_samples = entity_->topic_info()->qos_.depth;
    if (entity_->topic_info()->qos_.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      adv_sub_opts.recovery = AdvancedSubscriberOptions::RecoveryOptions{};
      adv_sub_opts.recovery->last_sample_miss_detection = RecoveryOptions::Heartbeat{};
    }
  }

  if (publisher_info.has_value()) {
    rmw_gid_t publisher_gid = {};
    std::memcpy(publisher_gid.data, publisher_info->info.endpoint_gid, RMW_GID_STORAGE_SIZE);
    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] Creating endpoint-aware subscription for key='%s' (publisher gid=%s)",
      key.c_str(),
      gid_to_hex(publisher_gid).c_str());
  } else {
    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] Creating CPU-group subscription for key='%s'",
      key.c_str());
  }

  auto endpoint = std::make_shared<SubscriptionEndpoint>();
  endpoint->key = key;
  endpoint->publisher_info = std::move(publisher_info);
  endpoint->is_accelerated = is_accelerated;

  std::weak_ptr<SubscriptionData> data_wp = shared_from_this();
  auto ep = endpoint;
  auto on_sample = [data_wp, ep, key](const zenoh::Sample & sample) {
      auto sub_data = data_wp.lock();
      if (sub_data == nullptr) {
        return;
      }

      RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp",
        "[Subscription] Received sample on key='%s' (sample key='%s')",
        key.c_str(),
        std::string(sample.get_keyexpr().as_string_view()).c_str());

      auto attachment = sample.get_attachment();
      if (!attachment.has_value()) {
        RMW_ZENOH_ROSIDL_BUFFER_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Unable to obtain attachment for topic '%s'",
          std::string(sample.get_keyexpr().as_string_view()).c_str());
        return;
      }

      AttachmentData attachment_data(attachment.value());

      // For the shared accelerated channel, resolve the sending publisher's
      // endpoint info by looking up its GID from the attachment.  This lets
      // take_one_message use endpoint-aware deserialization without needing
      // a per-publisher zenoh subscriber.
      std::optional<EndpointInfoStorage> resolved_endpoint_info = ep->publisher_info;
      if (ep->is_accelerated) {
        resolved_endpoint_info = sub_data->lookup_publisher_endpoint_info(
          attachment_data.copy_gid());
        if (!resolved_endpoint_info.has_value()) {
          // Publisher liveliness discovery may not have caught up yet.  Fall
          // back to a minimal endpoint_info populated with only the GID so
          // endpoint-aware deserialization still runs (matches the behavior
          // of rmw_fastrtps_cpp's buffer-aware take path).
          EndpointInfoStorage minimal;
          minimal.info = rmw_get_zero_initialized_topic_endpoint_info();
          minimal.info.endpoint_type = RMW_ENDPOINT_PUBLISHER;
          const auto gid_bytes = attachment_data.copy_gid();
          std::memcpy(
            minimal.info.endpoint_gid, gid_bytes.data(), RMW_GID_STORAGE_SIZE);
          resolved_endpoint_info = std::move(minimal);
        }
      }

      sub_data->add_new_message(
        std::make_unique<SubscriptionData::Message>(
          sample.get_payload(),
          get_system_time_in_ns(),
          std::move(attachment_data),
          std::move(resolved_endpoint_info)),
        std::string(sample.get_keyexpr().as_string_view()));
    };

  auto sub = context_impl->session()->ext().declare_advanced_subscriber(
    sub_ke,
    std::move(on_sample),
    zenoh::closures::none,
    std::move(adv_sub_opts),
    &result);

  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create zenoh subscription for key: %s", key.c_str());
    return nullptr;
  }

  endpoint->sub = std::optional<zenoh::ext::AdvancedSubscriber<void>>(std::move(sub));

  RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[Subscription] Created buffer-aware subscription for key: '%s'",
    key.c_str());

  return endpoint;
}

///=============================================================================
rmw_ret_t SubscriptionData::shutdown()
{
  rmw_ret_t ret = RMW_RET_OK;
  std::unordered_map<std::string, std::shared_ptr<SubscriptionEndpoint>> endpoints_to_destroy;
  bool was_buffer_aware = false;
  std::optional<zenoh::ext::AdvancedSubscriber<void>> base_sub;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_shutdown_ || !initialized_) {
      return ret;
    }
    is_shutdown_ = true;
    initialized_ = false;
    was_buffer_aware = is_buffer_aware_;
    if (was_buffer_aware) {
      endpoints_to_destroy = std::move(sub_endpoints_);
      sub_endpoints_.clear();
    }
    if (sub_.has_value()) {
      base_sub = std::move(sub_);
      sub_.reset();
    }
  }

  graph_cache_->remove_qos_event_callbacks(entity_->gid_hash());

  if (was_buffer_aware) {
    graph_cache_->unregister_discovery_callbacks(entity_->gid_hash());
  }

  zenoh::ZResult result;
  std::move(token_).value().undeclare(&result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to undeclare the liveliness token for topic '%s'",
      entity_->topic_info().value().name_.c_str());
    return RMW_RET_ERROR;
  }

  for (auto & [key, endpoint] : endpoints_to_destroy) {
    if (endpoint->sub.has_value()) {
      std::move(endpoint->sub.value()).undeclare(&result);
      if (result != Z_OK) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Unable to undeclare subscription for key '%s'",
          key.c_str());
        ret = RMW_RET_ERROR;
      }
    }
  }

  if (base_sub.has_value()) {
    std::move(base_sub.value()).undeclare(&result);
    if (result != Z_OK) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Unable to undeclare the subscriber for topic '%s'",
        entity_->topic_info().value().name_.c_str());
      return RMW_RET_ERROR;
    }
  }

  sess_.reset();
  return ret;
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
  if (entity_ && entity_->topic_info().has_value()) {
    RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] take_one_message topic='%s' is_buffer_aware=%d queue_size=%zu",
      entity_->topic_info().value().name_.c_str(),
      is_buffer_aware_,
      message_queue_.size());
  }
  if (is_shutdown_ || message_queue_.empty()) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  std::unique_ptr<Message> msg_data = std::move(message_queue_.front());
  message_queue_.pop_front();

  const Payload & payload_data = msg_data->payload;

  if (payload_data.empty()) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "SubscriptionData not able to get slice data");
    return RMW_RET_ERROR;
  }

  // Object that manages the raw buffer
  // FastCDR needs extra space for internal operations during deserialization
  // Allocate a larger buffer and copy the payload data
  // TODO(wjwwood): Use actual serialized message size instead of conservative estimate
  size_t buffer_size =
    payload_data.size() * 4 + 65536;  // 4x + 64KB safety margin
  rcutils_allocator_t * allocator = &rmw_node_->context->options.allocator;
  void * buffer_data = allocator->allocate(buffer_size, allocator->state);
  if (buffer_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate deserialization buffer");
    return RMW_RET_ERROR;
  }
  auto cleanup_buffer = rcpputils::make_scope_exit(
    [allocator, buffer_data]() {
      allocator->deallocate(buffer_data, allocator->state);
    });

  // Copy payload data to the larger buffer
  std::memcpy(buffer_data, payload_data.data(), payload_data.size());

  // FastCDR needs to know the actual data size, not the buffer size
  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(buffer_data),
    payload_data.size());  // Use actual payload size, not allocated buffer size

  // Object that deserializes the data
  rmw_zenoh_cpp::Cdr deser(fastbuffer);

  bool deserialize_success = false;

  try {
    if (msg_data->endpoint_info.has_value()) {
      // Message arrived on a per-publisher endpoint key -- use endpoint-aware
      // deserialization so vendor-backed buffer descriptors are resolved.
      rmw_context_impl_t * ctx_impl =
        static_cast<rmw_context_impl_t *>(rmw_node_->context->impl);
      auto * backend_ctx = ctx_impl->buffer_backend_context();
      if (!backend_ctx) {
        RMW_SET_ERROR_MSG("Buffer-aware deserialize missing buffer backend context");
        return RMW_RET_ERROR;
      }
      deserialize_success = type_support_->deserialize_ros_message_with_endpoint(
        deser.get_cdr(),
        ros_message,
        type_support_impl_,
        msg_data->endpoint_info->info,
        backend_ctx->serialization_context);
    } else {
      // Message arrived on the base key (legacy / standard serialization).
      deserialize_success = type_support_->deserialize_ros_message(
        deser.get_cdr(),
        ros_message,
        type_support_impl_);
    }
  } catch (const std::exception & e) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "[Subscription] EXCEPTION during deserialization: %s", e.what());
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("Deserialization exception: %s", e.what());
    return RMW_RET_ERROR;
  }

  if (!deserialize_success) {
    RMW_SET_ERROR_MSG("could not deserialize ROS message");
    return RMW_RET_ERROR;
  }

  if (message_info != nullptr) {
    message_info->source_timestamp = msg_data->attachment.source_timestamp();
    message_info->received_timestamp = msg_data->recv_timestamp;
    message_info->publication_sequence_number = msg_data->attachment.sequence_number();
    message_info->reception_sequence_number =
      reception_sn_.fetch_add(1, std::memory_order_relaxed);
    message_info->publisher_gid.implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
    memcpy(
      message_info->publisher_gid.data,
      msg_data->attachment.copy_gid().data(),
      RMW_GID_STORAGE_SIZE);
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

  const Payload & payload_data = msg_data->payload;

  if (payload_data.empty()) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "SubscriptionData not able to get slice data");
    return RMW_RET_ERROR;
  }
  if (serialized_message->buffer_capacity < payload_data.size()) {
    rmw_ret_t ret =
      rmw_serialized_message_resize(serialized_message, payload_data.size());
    if (ret != RMW_RET_OK) {
      return ret;  // Error message already set
    }
  }
  serialized_message->buffer_length = payload_data.size();
  memcpy(
    serialized_message->buffer,
    payload_data.data(),
    payload_data.size());

  *taken = true;

  if (message_info != nullptr) {
    message_info->source_timestamp = msg_data->attachment.source_timestamp();
    message_info->received_timestamp = msg_data->recv_timestamp;
    message_info->publication_sequence_number = msg_data->attachment.sequence_number();
    message_info->reception_sequence_number =
      reception_sn_.fetch_add(1, std::memory_order_relaxed);
    message_info->publisher_gid.implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
    memcpy(
      message_info->publisher_gid.data,
      msg_data->attachment.copy_gid().data(),
      RMW_GID_STORAGE_SIZE);
    message_info->from_intra_process = false;
  }

  return RMW_RET_OK;
}

///=============================================================================
void SubscriptionData::add_new_message(
  std::unique_ptr<SubscriptionData::Message> msg,
  const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }
  RMW_ZENOH_ROSIDL_BUFFER_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[Subscription] add_new_message topic='%s' is_buffer_aware=%d payload_size=%zu",
    topic_name.c_str(),
    is_buffer_aware_,
    msg->payload.size());
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
  const size_t gid_hash = hash_gid(msg->attachment.copy_gid());
  auto last_known_pub_it = last_known_published_msg_.find(gid_hash);
  if (last_known_pub_it != last_known_published_msg_.end()) {
    const int64_t seq_increment = std::abs(
      msg->attachment.sequence_number() -
      last_known_pub_it->second);
    if (seq_increment > 1) {
      int32_t num_msg_lost =
        static_cast<int32_t>(std::clamp(
          seq_increment - 1,
          static_cast<int64_t>(std::numeric_limits<int32_t>::min()),
          static_cast<int64_t>(std::numeric_limits<int32_t>::max())));
      events_mgr_->update_event_status(
        ZENOH_EVENT_MESSAGE_LOST,
        std::move(num_msg_lost));
    }
  }
  // Always update the last known sequence number for the publisher.
  last_known_published_msg_[gid_hash] = msg->attachment.sequence_number();

  message_queue_.emplace_back(std::move(msg));

  // Since we added new data, trigger user callback and guard condition if they are available
  data_callback_mgr_.trigger_callback();
  if (wait_set_data_ != nullptr) {
    std::lock_guard<std::mutex> wait_set_lock(wait_set_data_->condition_mutex);
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
  data_callback_mgr_.set_callback(user_data, std::move(callback));
}

//==============================================================================
std::shared_ptr<GraphCache> SubscriptionData::graph_cache() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_cache_;
}

}  // namespace rmw_zenoh_cpp
