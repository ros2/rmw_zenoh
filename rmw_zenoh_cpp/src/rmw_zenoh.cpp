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

#include <fastcdr/FastBuffer.h>

#include <zenoh.h>

#include <chrono>
#include <cinttypes>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <random>
#include <string>
#include <utility>

#include "detail/attachment_helpers.hpp"
#include "detail/cdr.hpp"
#include "detail/guard_condition.hpp"
#include "detail/graph_cache.hpp"
#include "detail/identifier.hpp"
#include "detail/liveliness_utils.hpp"
#include "detail/logging_macros.hpp"
#include "detail/message_type_support.hpp"
#include "detail/qos.hpp"
#include "detail/rmw_data_types.hpp"
#include "detail/serialization_format.hpp"
#include "detail/type_support_common.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rcutils/env.h"
#include "rcutils/strdup.h"
#include "rcutils/types.h"

#include "rmw/allocators.h"
#include "rmw/dynamic_message_type_support.h"
#include "rmw/error_handling.h"
#include "rmw/features.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

namespace
{
//==============================================================================
// Helper function to create a copy of a string after removing any
// leading or trailing slashes.
std::string strip_slashes(const char * const str)
{
  std::string ret = std::string(str);
  const std::size_t len = strlen(str);
  std::size_t start = 0;
  std::size_t end = len - 1;
  if (str[0] == '/') {
    ++start;
  }
  if (str[end] == '/') {
    --end;
  }
  return ret.substr(start, end - start + 1);
}

//==============================================================================
// A function that generates a key expression for message transport of the format
// <ros_domain_id>/<topic_name>/<topic_type>/<topic_hash>
// In particular, Zenoh keys cannot start or end with a /, so this function
// will strip them out.
// Performance note: at present, this function allocates a new string and copies
// the old string into it. If this becomes a performance problem, we could consider
// modifying the topic_name in place. But this means we need to be much more
// careful about who owns the string.
z_owned_keyexpr_t ros_topic_name_to_zenoh_key(
  const std::size_t domain_id,
  const char * const topic_name,
  const char * const topic_type,
  const char * const type_hash)
{
  std::string keyexpr_str = std::to_string(domain_id);
  keyexpr_str += "/";
  keyexpr_str += strip_slashes(topic_name);
  keyexpr_str += "/";
  keyexpr_str += topic_type;
  keyexpr_str += "/";
  keyexpr_str += type_hash;

  return z_keyexpr_new(keyexpr_str.c_str());
}

//==============================================================================
const rosidl_message_type_support_t * find_message_type_support(
  const rosidl_message_type_support_t * type_supports)
{
  const rosidl_message_type_support_t * type_support = get_message_typesupport_handle(
    type_supports, RMW_ZENOH_CPP_TYPESUPPORT_C);
  if (!type_support) {
    rcutils_error_string_t prev_error_string = rcutils_get_error_string();
    rcutils_reset_error();
    type_support = get_message_typesupport_handle(type_supports, RMW_ZENOH_CPP_TYPESUPPORT_CPP);
    if (!type_support) {
      rcutils_error_string_t error_string = rcutils_get_error_string();
      rcutils_reset_error();
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "Type support not from this implementation. Got:\n"
        "    %s\n"
        "    %s\n"
        "while fetching it",
        prev_error_string.str, error_string.str);
      return nullptr;
    }
  }

  return type_support;
}

//==============================================================================
const rosidl_service_type_support_t * find_service_type_support(
  const rosidl_service_type_support_t * type_supports)
{
  const rosidl_service_type_support_t * type_support = get_service_typesupport_handle(
    type_supports, RMW_ZENOH_CPP_TYPESUPPORT_C);
  if (!type_support) {
    rcutils_error_string_t prev_error_string = rcutils_get_error_string();
    rcutils_reset_error();
    type_support = get_service_typesupport_handle(
      type_supports, RMW_ZENOH_CPP_TYPESUPPORT_CPP);
    if (!type_support) {
      rcutils_error_string_t error_string = rcutils_get_error_string();
      rcutils_reset_error();
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "Type support not from this implementation. Got:\n"
        "    %s\n"
        "    %s\n"
        "while fetching it",
        prev_error_string.str, error_string.str);
      return nullptr;
    }
  }

  return type_support;
}

}  // namespace

extern "C"
{
//==============================================================================
/// Get the name of the rmw implementation being used
const char *
rmw_get_implementation_identifier(void)
{
  return rmw_zenoh_cpp::rmw_zenoh_identifier;
}

//==============================================================================
/// Get the unique serialization format for this middleware.
const char *
rmw_get_serialization_format(void)
{
  return rmw_zenoh_cpp::rmw_zenoh_serialization_format;
}

//==============================================================================
bool rmw_feature_supported(rmw_feature_t feature)
{
  switch (feature) {
    case RMW_FEATURE_MESSAGE_INFO_PUBLICATION_SEQUENCE_NUMBER:
      return false;
    case RMW_FEATURE_MESSAGE_INFO_RECEPTION_SEQUENCE_NUMBER:
      return false;
    case RMW_MIDDLEWARE_SUPPORTS_TYPE_DISCOVERY:
      return true;
    case RMW_MIDDLEWARE_CAN_TAKE_DYNAMIC_MESSAGE:
      return false;
    default:
      return false;
  }
}

//==============================================================================
/// Create a node and return a handle to that node.
rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl->enclave,
    "expected initialized enclave",
    return nullptr);
  if (context->impl->is_shutdown) {
    RCUTILS_SET_ERROR_MSG("context has been shutdown");
    return nullptr;
  }

  int validation_result = RMW_NODE_NAME_VALID;
  rmw_ret_t ret = rmw_validate_node_name(name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  if (RMW_NODE_NAME_VALID != validation_result) {
    const char * reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node name: %s", reason);
    return nullptr;
  }

  validation_result = RMW_NAMESPACE_VALID;
  ret = rmw_validate_namespace(namespace_, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  if (RMW_NAMESPACE_VALID != validation_result) {
    const char * reason = rmw_namespace_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node namespace: %s", reason);
    return nullptr;
  }

  rcutils_allocator_t * allocator = &context->options.allocator;

  rmw_node_t * node =
    static_cast<rmw_node_t *>(allocator->zero_allocate(1, sizeof(rmw_node_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node,
    "unable to allocate memory for rmw_node_t",
    return nullptr);
  auto free_node = rcpputils::make_scope_exit(
    [node, allocator]() {
      allocator->deallocate(node, allocator->state);
    });

  node->name = rcutils_strdup(name, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->name,
    "unable to allocate memory for node name",
    return nullptr);
  auto free_name = rcpputils::make_scope_exit(
    [node, allocator]() {
      allocator->deallocate(const_cast<char *>(node->name), allocator->state);
    });

  node->namespace_ = rcutils_strdup(namespace_, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->namespace_,
    "unable to allocate memory for node namespace",
    return nullptr);
  auto free_namespace = rcpputils::make_scope_exit(
    [node, allocator]() {
      allocator->deallocate(const_cast<char *>(node->namespace_), allocator->state);
    });

  // Put metadata into node->data.
  auto node_data = static_cast<rmw_zenoh_cpp::rmw_node_data_t *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::rmw_node_data_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data,
    "failed to allocate memory for node data",
    return nullptr);
  auto free_node_data = rcpputils::make_scope_exit(
    [node_data, allocator]() {
      allocator->deallocate(node_data, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(
    node_data, node_data, return nullptr,
    rmw_zenoh_cpp::rmw_node_data_t);
  auto destruct_node_data = rcpputils::make_scope_exit(
    [node_data]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        node_data->~rmw_node_data_t(), rmw_zenoh_cpp::rmw_node_data_t);
    });

  // Initialize liveliness token for the node to advertise that a new node is in town.
  node_data->id = context->impl->get_next_entity_id();
  node_data->entity = rmw_zenoh_cpp::liveliness::Entity::make(
    z_info_zid(z_loan(context->impl->session)),
    std::to_string(node_data->id),
    std::to_string(node_data->id),
    rmw_zenoh_cpp::liveliness::EntityType::Node,
    rmw_zenoh_cpp::liveliness::NodeInfo{context->actual_domain_id, namespace_, name,
      context->impl->enclave});
  if (node_data->entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the node.");
    return nullptr;
  }
  node_data->token = zc_liveliness_declare_token(
    z_loan(context->impl->session),
    z_keyexpr(node_data->entity->keyexpr().c_str()),
    NULL
  );
  auto free_token = rcpputils::make_scope_exit(
    [node_data]() {
      z_drop(z_move(node_data->token));
    });
  if (!z_check(node_data->token)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the node.");
    return nullptr;
  }

  node->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  node->context = context;
  node->data = node_data;

  free_token.cancel();
  free_node_data.cancel();
  destruct_node_data.cancel();
  free_namespace.cancel();
  free_name.cancel();
  free_node.cancel();
  return node;
}

//==============================================================================
/// Finalize a given node handle, reclaim the resources, and deallocate the node handle.
rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // Undeclare liveliness token for the node to advertise that the node has ridden
  // off into the sunset.
  rmw_zenoh_cpp::rmw_node_data_t * node_data =
    static_cast<rmw_zenoh_cpp::rmw_node_data_t *>(node->data);
  if (node_data != nullptr) {
    zc_liveliness_undeclare_token(z_move(node_data->token));
    RMW_TRY_DESTRUCTOR(node_data->~rmw_node_data_t(), rmw_node_data_t, );
    allocator->deallocate(node_data, allocator->state);
  }

  allocator->deallocate(const_cast<char *>(node->namespace_), allocator->state);
  allocator->deallocate(const_cast<char *>(node->name), allocator->state);
  allocator->deallocate(node, allocator->state);

  return RMW_RET_OK;
}

//==============================================================================
/// Return a guard condition which is triggered when the ROS graph changes.
const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(const rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);

  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, nullptr);

  return node->context->impl->graph_guard_condition;
}

//==============================================================================
/// Initialize a publisher allocation to be used with later publications.
rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_publisher_allocation_t * allocation)
{
  static_cast<void>(type_support);
  static_cast<void>(message_bounds);
  static_cast<void>(allocation);
  RMW_SET_ERROR_MSG("rmw_init_publisher_allocation: unimplemented");
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Destroy a publisher allocation object.
rmw_ret_t
rmw_fini_publisher_allocation(
  rmw_publisher_allocation_t * allocation)
{
  static_cast<void>(allocation);
  RMW_SET_ERROR_MSG("rmw_fini_publisher_allocation: unimplemented");
  return RMW_RET_UNSUPPORTED;
}

namespace
{
void
generate_random_gid(uint8_t gid[RMW_GID_STORAGE_SIZE])
{
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_int_distribution<std::mt19937::result_type> dist(
    std::numeric_limits<unsigned char>::min(), std::numeric_limits<unsigned char>::max());

  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    gid[i] = dist(rng);
  }
}
}  // namespace

//==============================================================================
/// Create a publisher and return a handle to that publisher.
rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  if (topic_name[0] == '\0') {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  if (!qos_profile->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic name: %s", reason);
      return nullptr;
    }
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);
  if (publisher_options->require_unique_network_flow_endpoints ==
    RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED)
  {
    RMW_SET_ERROR_MSG(
      "Strict requirement on unique network flow endpoints for publishers not supported");
    return nullptr;
  }
  const rmw_zenoh_cpp::rmw_node_data_t * node_data =
    static_cast<rmw_zenoh_cpp::rmw_node_data_t *>(node->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_data, nullptr);

  // Get the RMW type support.
  const rosidl_message_type_support_t * type_support = find_message_type_support(type_supports);
  if (type_support == nullptr) {
    // error was already set by find_message_type_support
    return nullptr;
  }

  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return nullptr);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(
    node->context->impl);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl,
    "unable to get rmw_context_impl_s",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl->enclave,
    "expected initialized enclave",
    return nullptr);
  if (!z_check(context_impl->session)) {
    RMW_SET_ERROR_MSG("zenoh session is invalid");
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // Create the publisher.
  auto rmw_publisher =
    static_cast<rmw_publisher_t *>(allocator->zero_allocate(
      1, sizeof(rmw_publisher_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_publisher,
    "failed to allocate memory for the publisher",
    return nullptr);
  auto free_rmw_publisher = rcpputils::make_scope_exit(
    [rmw_publisher, allocator]() {
      allocator->deallocate(rmw_publisher, allocator->state);
    });

  auto publisher_data = static_cast<rmw_zenoh_cpp::rmw_publisher_data_t *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::rmw_publisher_data_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher_data,
    "failed to allocate memory for publisher data",
    return nullptr);
  auto free_publisher_data = rcpputils::make_scope_exit(
    [publisher_data, allocator]() {
      allocator->deallocate(publisher_data, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(
    publisher_data, publisher_data, return nullptr,
    rmw_zenoh_cpp::rmw_publisher_data_t);
  auto destruct_publisher_data = rcpputils::make_scope_exit(
    [publisher_data]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        publisher_data->~rmw_publisher_data_t(), rmw_zenoh_cpp::rmw_publisher_data_t);
    });

  generate_random_gid(publisher_data->pub_gid);

  // Adapt any 'best available' QoS options
  publisher_data->adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = rmw_zenoh_cpp::QoS::get().best_available_qos(
    node, topic_name, &publisher_data->adapted_qos_profile, rmw_get_subscriptions_info_by_topic);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }

  publisher_data->typesupport_identifier = type_support->typesupport_identifier;
  publisher_data->type_hash = type_support->get_type_hash_func(type_support);
  publisher_data->type_support_impl = type_support->data;
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);
  publisher_data->type_support = static_cast<rmw_zenoh_cpp::MessageTypeSupport *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::MessageTypeSupport), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher_data->type_support,
    "Failed to allocate MessageTypeSupport",
    return nullptr);
  auto free_type_support = rcpputils::make_scope_exit(
    [publisher_data, allocator]() {
      allocator->deallocate(publisher_data->type_support, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(
    publisher_data->type_support,
    publisher_data->type_support,
    return nullptr,
    rmw_zenoh_cpp::MessageTypeSupport, callbacks);
  auto destruct_msg_type_support = rcpputils::make_scope_exit(
    [publisher_data]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        publisher_data->type_support->~MessageTypeSupport(),
        rmw_zenoh_cpp::MessageTypeSupport);
    });

  publisher_data->context = node->context;
  rmw_publisher->data = publisher_data;
  rmw_publisher->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  rmw_publisher->options = *publisher_options;
  rmw_publisher->can_loan_messages = false;

  rmw_publisher->topic_name = rcutils_strdup(topic_name, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_publisher->topic_name,
    "Failed to allocate topic name",
    return nullptr);
  auto free_topic_name = rcpputils::make_scope_exit(
    [rmw_publisher, allocator]() {
      allocator->deallocate(const_cast<char *>(rmw_publisher->topic_name), allocator->state);
    });

  // Convert the type hash to a string so that it can be included in
  // the keyexpr.
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(
    publisher_data->type_hash,
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

  z_owned_keyexpr_t keyexpr = ros_topic_name_to_zenoh_key(
    node->context->actual_domain_id,
    topic_name,
    publisher_data->type_support->get_name(),
    type_hash_c_str);
  auto always_free_ros_keyexpr = rcpputils::make_scope_exit(
    [&keyexpr]() {
      z_keyexpr_drop(z_move(keyexpr));
    });
  if (!z_keyexpr_check(&keyexpr)) {
    RMW_SET_ERROR_MSG("unable to create zenoh keyexpr.");
    return nullptr;
  }

  // Create a Publication Cache if durability is transient_local.
  if (publisher_data->adapted_qos_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    ze_publication_cache_options_t pub_cache_opts = ze_publication_cache_options_default();
    pub_cache_opts.history = publisher_data->adapted_qos_profile.depth;
    pub_cache_opts.queryable_complete = true;
    publisher_data->pub_cache = ze_declare_publication_cache(
      z_loan(context_impl->session),
      z_loan(keyexpr),
      &pub_cache_opts
    );
    if (!publisher_data->pub_cache.has_value() || !z_check(publisher_data->pub_cache.value())) {
      RMW_SET_ERROR_MSG("unable to create zenoh publisher cache");
      return nullptr;
    }
  }
  auto undeclare_z_publisher_cache = rcpputils::make_scope_exit(
    [publisher_data]() {
      if (publisher_data && publisher_data->pub_cache.has_value()) {
        z_drop(z_move(publisher_data->pub_cache.value()));
      }
    });

  // Set congestion_control to BLOCK if appropriate.
  z_publisher_options_t opts = z_publisher_options_default();
  opts.congestion_control = Z_CONGESTION_CONTROL_DROP;
  if (publisher_data->adapted_qos_profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
    publisher_data->adapted_qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE)
  {
    opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
  }
  // TODO(clalancette): What happens if the key name is a valid but empty string?
  publisher_data->pub = z_declare_publisher(
    z_loan(context_impl->session),
    z_loan(keyexpr),
    &opts
  );
  if (!z_check(publisher_data->pub)) {
    RMW_SET_ERROR_MSG("unable to create zenoh publisher");
    return nullptr;
  }
  auto undeclare_z_publisher = rcpputils::make_scope_exit(
    [publisher_data]() {
      z_undeclare_publisher(z_move(publisher_data->pub));
    });

  publisher_data->entity = rmw_zenoh_cpp::liveliness::Entity::make(
    z_info_zid(z_loan(node->context->impl->session)),
    std::to_string(node_data->id),
    std::to_string(
      context_impl->get_next_entity_id()),
    rmw_zenoh_cpp::liveliness::EntityType::Publisher,
    rmw_zenoh_cpp::liveliness::NodeInfo{
      node->context->actual_domain_id, node->namespace_, node->name, context_impl->enclave},
    rmw_zenoh_cpp::liveliness::TopicInfo{
      rmw_publisher->topic_name,
      publisher_data->type_support->get_name(),
      type_hash_c_str,
      publisher_data->adapted_qos_profile}
  );
  if (publisher_data->entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the publisher.");
    return nullptr;
  }
  publisher_data->token = zc_liveliness_declare_token(
    z_loan(node->context->impl->session),
    z_keyexpr(publisher_data->entity->keyexpr().c_str()),
    NULL
  );
  auto free_token = rcpputils::make_scope_exit(
    [publisher_data]() {
      if (publisher_data != nullptr) {
        z_drop(z_move(publisher_data->token));
      }
    });
  if (!z_check(publisher_data->token)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the publisher.");
    return nullptr;
  }

  free_token.cancel();
  undeclare_z_publisher_cache.cancel();
  undeclare_z_publisher.cancel();
  free_topic_name.cancel();
  destruct_msg_type_support.cancel();
  free_type_support.cancel();
  destruct_publisher_data.cancel();
  free_publisher_data.cancel();
  free_rmw_publisher.cancel();

  return rmw_publisher;
}

//==============================================================================
/// Finalize a given publisher handle, reclaim the resources, and deallocate the publisher handle.
rmw_ret_t
rmw_destroy_publisher(rmw_node_t * node, rmw_publisher_t * publisher)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rmw_ret_t ret = RMW_RET_OK;

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  auto publisher_data = static_cast<rmw_zenoh_cpp::rmw_publisher_data_t *>(publisher->data);
  if (publisher_data != nullptr) {
    zc_liveliness_undeclare_token(z_move(publisher_data->token));
    if (publisher_data->pub_cache.has_value()) {
      z_drop(z_move(publisher_data->pub_cache.value()));
    }
    RMW_TRY_DESTRUCTOR(publisher_data->type_support->~MessageTypeSupport(), MessageTypeSupport, );
    allocator->deallocate(publisher_data->type_support, allocator->state);
    if (z_undeclare_publisher(z_move(publisher_data->pub))) {
      RMW_SET_ERROR_MSG("failed to undeclare pub");
      ret = RMW_RET_ERROR;
    }
    RMW_TRY_DESTRUCTOR(publisher_data->~rmw_publisher_data_t(), rmw_publisher_data_t, );
    allocator->deallocate(publisher_data, allocator->state);
  }
  allocator->deallocate(const_cast<char *>(publisher->topic_name), allocator->state);
  allocator->deallocate(publisher, allocator->state);

  return ret;
}

//==============================================================================
rmw_ret_t
rmw_take_dynamic_message(
  const rmw_subscription_t * subscription,
  rosidl_dynamic_typesupport_dynamic_data_t * dynamic_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(subscription);
  static_cast<void>(dynamic_message);
  static_cast<void>(taken);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
rmw_ret_t
rmw_take_dynamic_message_with_info(
  const rmw_subscription_t * subscription,
  rosidl_dynamic_typesupport_dynamic_data_t * dynamic_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(subscription);
  static_cast<void>(dynamic_message);
  static_cast<void>(taken);
  static_cast<void>(message_info);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
rmw_ret_t
rmw_serialization_support_init(
  const char * serialization_lib_name,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support)
{
  static_cast<void>(serialization_lib_name);
  static_cast<void>(allocator);
  static_cast<void>(serialization_support);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Borrow a loaned ROS message.
rmw_ret_t
rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message)
{
  static_cast<void>(publisher);
  static_cast<void>(type_support);
  static_cast<void>(ros_message);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Return a loaned message previously borrowed from a publisher.
rmw_ret_t
rmw_return_loaned_message_from_publisher(
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  static_cast<void>(publisher);
  static_cast<void>(loaned_message);
  return RMW_RET_UNSUPPORTED;
}

namespace
{
z_owned_bytes_map_t
create_map_and_set_sequence_num(int64_t sequence_number, uint8_t gid[RMW_GID_STORAGE_SIZE])
{
  z_owned_bytes_map_t map = z_bytes_map_new();
  if (!z_check(map)) {
    RMW_SET_ERROR_MSG("failed to allocate map for sequence number");
    return z_bytes_map_null();
  }
  auto free_attachment_map = rcpputils::make_scope_exit(
    [&map]() {
      z_bytes_map_drop(z_move(map));
    });

  // The largest possible int64_t number is INT64_MAX, i.e. 9223372036854775807.
  // That is 19 characters long, plus one for the trailing \0, means we need 20 bytes.
  char seq_id_str[20];
  if (rcutils_snprintf(seq_id_str, sizeof(seq_id_str), "%" PRId64, sequence_number) < 0) {
    RMW_SET_ERROR_MSG("failed to print sequence_number into buffer");
    return z_bytes_map_null();
  }
  z_bytes_map_insert_by_copy(&map, z_bytes_new("sequence_number"), z_bytes_new(seq_id_str));

  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  char source_ts_str[20];
  if (rcutils_snprintf(source_ts_str, sizeof(source_ts_str), "%" PRId64, now_ns.count()) < 0) {
    RMW_SET_ERROR_MSG("failed to print sequence_number into buffer");
    return z_bytes_map_null();
  }
  z_bytes_map_insert_by_copy(&map, z_bytes_new("source_timestamp"), z_bytes_new(source_ts_str));

  z_bytes_t gid_bytes;
  gid_bytes.len = RMW_GID_STORAGE_SIZE;
  gid_bytes.start = gid;

  z_bytes_map_insert_by_copy(&map, z_bytes_new("source_gid"), gid_bytes);

  free_attachment_map.cancel();

  return map;
}
}  // namespace

//==============================================================================
/// Publish a ROS message.
rmw_ret_t
rmw_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  static_cast<void>(allocation);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher, "publisher handle is null",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher, publisher->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    ros_message, "ros message handle is null",
    return RMW_RET_INVALID_ARGUMENT);

  auto publisher_data = static_cast<rmw_zenoh_cpp::rmw_publisher_data_t *>(publisher->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher_data, "publisher_data is null",
    return RMW_RET_INVALID_ARGUMENT);

  rcutils_allocator_t * allocator = &(publisher_data->context->options.allocator);

  // Serialize data.
  size_t max_data_length = publisher_data->type_support->get_estimated_serialized_size(
    ros_message,
    publisher_data->type_support_impl);

  // To store serialized message byte array.
  char * msg_bytes = nullptr;
  std::optional<zc_owned_shmbuf_t> shmbuf = std::nullopt;
  auto always_free_shmbuf = rcpputils::make_scope_exit(
    [&shmbuf]() {
      if (shmbuf.has_value()) {
        zc_shmbuf_drop(&shmbuf.value());
      }
    });
  auto free_msg_bytes = rcpputils::make_scope_exit(
    [&msg_bytes, allocator, &shmbuf]() {
      if (msg_bytes && !shmbuf.has_value()) {
        allocator->deallocate(msg_bytes, allocator->state);
      }
    });

  // Get memory from SHM buffer if available.
  if (publisher_data->context->impl->shm_manager.has_value() &&
    zc_shm_manager_check(&publisher_data->context->impl->shm_manager.value()))
  {
    shmbuf = zc_shm_alloc(
      &publisher_data->context->impl->shm_manager.value(),
      max_data_length);
    if (!z_check(shmbuf.value())) {
      zc_shm_gc(&publisher_data->context->impl->shm_manager.value());
      shmbuf = zc_shm_alloc(&publisher_data->context->impl->shm_manager.value(), max_data_length);
      if (!z_check(shmbuf.value())) {
        // TODO(Yadunund): Should we revert to regular allocation and not return an error?
        RMW_SET_ERROR_MSG("Failed to allocate a SHM buffer, even after GCing");
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
  if (!publisher_data->type_support->serialize_ros_message(
      ros_message,
      ser.get_cdr(),
      publisher_data->type_support_impl))
  {
    RMW_SET_ERROR_MSG("could not serialize ROS message");
    return RMW_RET_ERROR;
  }

  const size_t data_length = ser.get_serialized_data_length();

  int64_t sequence_number = publisher_data->get_next_sequence_number();

  z_owned_bytes_map_t map =
    create_map_and_set_sequence_num(sequence_number, publisher_data->pub_gid);
  if (!z_check(map)) {
    // create_map_and_set_sequence_num already set the error
    return RMW_RET_ERROR;
  }
  auto free_attachment_map = rcpputils::make_scope_exit(
    [&map]() {
      z_bytes_map_drop(z_move(map));
    });

  int ret;
  // The encoding is simply forwarded and is useful when key expressions in the
  // session use different encoding formats. In our case, all key expressions
  // will be encoded with CDR so it does not really matter.
  z_publisher_put_options_t options = z_publisher_put_options_default();
  options.attachment = z_bytes_map_as_attachment(&map);

  if (shmbuf.has_value()) {
    zc_shmbuf_set_length(&shmbuf.value(), data_length);
    zc_owned_payload_t payload = zc_shmbuf_into_payload(z_move(shmbuf.value()));
    ret = zc_publisher_put_owned(z_loan(publisher_data->pub), z_move(payload), &options);
  } else {
    // Returns 0 if success.
    ret = z_publisher_put(
      z_loan(publisher_data->pub),
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

//==============================================================================
/// Publish a loaned ROS message.
rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  static_cast<void>(publisher);
  static_cast<void>(ros_message);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Retrieve the number of matched subscriptions to a publisher.
rmw_ret_t
rmw_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_count, RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_publisher_data_t * pub_data =
    static_cast<rmw_zenoh_cpp::rmw_publisher_data_t *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data->context, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(pub_data->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);

  return context_impl->graph_cache->publisher_count_matched_subscriptions(
    publisher, subscription_count);
}

//==============================================================================
/// Retrieve the actual qos settings of the publisher.
rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_publisher_data_t * pub_data =
    static_cast<rmw_zenoh_cpp::rmw_publisher_data_t *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, RMW_RET_INVALID_ARGUMENT);

  *qos = pub_data->adapted_qos_profile;
  return RMW_RET_OK;
}

//==============================================================================
/// Publish a ROS message as a byte stream.
rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  static_cast<void>(allocation);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher, "publisher handle is null",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher, publisher->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    serialized_message, "serialized message handle is null",
    return RMW_RET_INVALID_ARGUMENT);

  auto publisher_data = static_cast<rmw_zenoh_cpp::rmw_publisher_data_t *>(publisher->data);
  RCUTILS_CHECK_FOR_NULL_WITH_MSG(
    publisher_data, "publisher data pointer is null",
    return RMW_RET_ERROR);

  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  rmw_zenoh_cpp::Cdr ser(buffer);
  if (!ser.get_cdr().jump(serialized_message->buffer_length)) {
    RMW_SET_ERROR_MSG("cannot correctly set serialized buffer");
    return RMW_RET_ERROR;
  }

  uint64_t sequence_number = publisher_data->get_next_sequence_number();

  z_owned_bytes_map_t map =
    create_map_and_set_sequence_num(sequence_number, publisher_data->pub_gid);

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
  // Returns 0 if success.
  int8_t ret = z_publisher_put(
    z_loan(publisher_data->pub),
    serialized_message->buffer,
    data_length,
    &options);

  if (ret) {
    RMW_SET_ERROR_MSG("unable to publish message");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

//==============================================================================
/// Compute the size of a serialized message.
rmw_ret_t
rmw_get_serialized_message_size(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  size_t * size)
{
  static_cast<void>(type_support);
  static_cast<void>(message_bounds);
  static_cast<void>(size);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Manually assert that this Publisher is alive (for RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
rmw_ret_t
rmw_publisher_assert_liveliness(const rmw_publisher_t * publisher)
{
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher, "publisher handle is null",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher->data, "publisher data is null",
    return RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::rmw_publisher_data_t * pub_data =
    static_cast<rmw_zenoh_cpp::rmw_publisher_data_t *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, RMW_RET_INVALID_ARGUMENT);

  if (!zc_liveliness_token_check(&pub_data->token)) {
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

//==============================================================================
/// Wait until all published message data is acknowledged or until the specified timeout elapses.
rmw_ret_t
rmw_publisher_wait_for_all_acked(
  const rmw_publisher_t * publisher,
  rmw_time_t wait_timeout)
{
  static_cast<void>(publisher);
  static_cast<void>(wait_timeout);

  // We are not currently tracking all published data, so we don't know what data is in flight that
  // we might have to wait for.  Even if we did start tracking it, we don't have insight into the
  // TCP stream that Zenoh is managing for us, so we couldn't guarantee this anyway.
  // Just lie to the upper layers and say that everything is working as expected.
  // We return OK rather than UNSUPPORTED so that certain upper-layer tests continue working.
  return RMW_RET_OK;
}

//==============================================================================
/// Serialize a ROS message into a rmw_serialized_message_t.
rmw_ret_t
rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  const rosidl_message_type_support_t * ts = find_message_type_support(type_support);
  if (ts == nullptr) {
    // error was already set by find_message_type_support
    return RMW_RET_ERROR;
  }

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);
  auto tss = rmw_zenoh_cpp::MessageTypeSupport(callbacks);
  auto data_length = tss.get_estimated_serialized_size(ros_message, callbacks);
  if (serialized_message->buffer_capacity < data_length) {
    if (rmw_serialized_message_resize(serialized_message, data_length) != RMW_RET_OK) {
      RMW_SET_ERROR_MSG("unable to dynamically resize serialized message");
      return RMW_RET_ERROR;
    }
  }

  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), data_length);
  rmw_zenoh_cpp::Cdr ser(buffer);

  auto ret = tss.serialize_ros_message(ros_message, ser.get_cdr(), callbacks);
  serialized_message->buffer_length = data_length;
  serialized_message->buffer_capacity = data_length;
  return ret == true ? RMW_RET_OK : RMW_RET_ERROR;
}

//==============================================================================
/// Deserialize a ROS message.
rmw_ret_t
rmw_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_support,
  void * ros_message)
{
  const rosidl_message_type_support_t * ts = find_message_type_support(type_support);
  if (ts == nullptr) {
    // error was already set by find_message_type_support
    return RMW_RET_ERROR;
  }

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);
  auto tss = rmw_zenoh_cpp::MessageTypeSupport(callbacks);
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  rmw_zenoh_cpp::Cdr deser(buffer);

  auto ret = tss.deserialize_ros_message(deser.get_cdr(), ros_message, callbacks);
  return ret == true ? RMW_RET_OK : RMW_RET_ERROR;
}

//==============================================================================
/// Initialize a subscription allocation to be used with later `take`s.
rmw_ret_t
rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_subscription_allocation_t * allocation)
{
  // Unused in current implementation.
  static_cast<void>(type_support);
  static_cast<void>(message_bounds);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Destroy a publisher allocation object.
rmw_ret_t
rmw_fini_subscription_allocation(
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Create a subscription and return a handle to that subscription.
rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_subscription_options_t * subscription_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  if (topic_name[0] == '\0') {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_options, nullptr);

  const rosidl_message_type_support_t * type_support = find_message_type_support(type_supports);
  if (type_support == nullptr) {
    // error was already set by find_message_type_support
    return nullptr;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(node->data, nullptr);
  auto node_data = static_cast<rmw_zenoh_cpp::rmw_node_data_t *>(node->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data, "unable to create subscription as node_data is invalid.",
    return nullptr);
  // TODO(yadunund): Check if a duplicate entry for the same topic name + topic type
  // is present in node_data->subscriptions and if so return error;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return nullptr);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(
    node->context->impl);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl,
    "unable to get rmw_context_impl_s",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl->enclave,
    "expected initialized enclave",
    return nullptr);
  if (!z_check(context_impl->session)) {
    RMW_SET_ERROR_MSG("zenoh session is invalid");
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // Create the rmw_subscription.
  rmw_subscription_t * rmw_subscription =
    static_cast<rmw_subscription_t *>(allocator->zero_allocate(
      1, sizeof(rmw_subscription_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_subscription,
    "failed to allocate memory for the subscription",
    return nullptr);
  auto free_rmw_subscription = rcpputils::make_scope_exit(
    [rmw_subscription, allocator]() {
      allocator->deallocate(rmw_subscription, allocator->state);
    });

  auto sub_data = static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::rmw_subscription_data_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    sub_data,
    "failed to allocate memory for subscription data",
    return nullptr);
  auto free_sub_data = rcpputils::make_scope_exit(
    [sub_data, allocator]() {
      allocator->deallocate(sub_data, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(sub_data, sub_data, return nullptr, rmw_zenoh_cpp::rmw_subscription_data_t);
  auto destruct_sub_data = rcpputils::make_scope_exit(
    [sub_data]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        sub_data->~rmw_subscription_data_t(),
        rmw_zenoh_cpp::rmw_subscription_data_t);
    });

  // Adapt any 'best available' QoS options
  sub_data->adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = rmw_zenoh_cpp::QoS::get().best_available_qos(
    node, topic_name, &sub_data->adapted_qos_profile, rmw_get_publishers_info_by_topic);
  if (RMW_RET_OK != ret) {
    RMW_SET_ERROR_MSG("Failed to obtain adapted_qos_profile.");
    return nullptr;
  }

  sub_data->typesupport_identifier = type_support->typesupport_identifier;
  sub_data->type_hash = type_support->get_type_hash_func(type_support);
  sub_data->type_support_impl = type_support->data;
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);
  sub_data->type_support = static_cast<rmw_zenoh_cpp::MessageTypeSupport *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::MessageTypeSupport), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    sub_data->type_support,
    "Failed to allocate MessageTypeSupport",
    return nullptr);
  auto free_type_support = rcpputils::make_scope_exit(
    [sub_data, allocator]() {
      allocator->deallocate(sub_data->type_support, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(
    sub_data->type_support,
    sub_data->type_support,
    return nullptr,
    rmw_zenoh_cpp::MessageTypeSupport, callbacks);
  auto destruct_msg_type_support = rcpputils::make_scope_exit(
    [sub_data]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        sub_data->type_support->~MessageTypeSupport(),
        rmw_zenoh_cpp::MessageTypeSupport);
    });

  sub_data->context = node->context;

  rmw_subscription->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;

  rmw_subscription->topic_name = rcutils_strdup(topic_name, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_subscription->topic_name,
    "Failed to allocate topic name",
    return nullptr);
  auto free_topic_name = rcpputils::make_scope_exit(
    [rmw_subscription, allocator]() {
      allocator->deallocate(const_cast<char *>(rmw_subscription->topic_name), allocator->state);
    });

  rmw_subscription->options = *subscription_options;
  rmw_subscription->can_loan_messages = false;
  rmw_subscription->is_cft_enabled = false;

  // Convert the type hash to a string so that it can be included in
  // the keyexpr.
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(
    sub_data->type_hash,
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
  // z_owned_closure_sample_t callback = z_closure(rmw_zenoh_cpp::sub_data_handler, nullptr, sub_data);
  z_owned_closure_sample_t callback;
  callback.context = (void *)sub_data;
  callback.call = rmw_zenoh_cpp::sub_data_handler;
  callback.drop = nullptr;
  z_owned_keyexpr_t keyexpr = ros_topic_name_to_zenoh_key(
    node->context->actual_domain_id,
    topic_name,
    sub_data->type_support->get_name(),
    type_hash_c_str);
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

  if (sub_data->adapted_qos_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    ze_querying_subscriber_options_t sub_options = ze_querying_subscriber_options_default();
    // Target all complete publication caches which are queryables.
    sub_options.query_target = Z_QUERY_TARGET_ALL_COMPLETE;
    // We set consolidation to none as we need to receive transient local messages
    // from a number of publishers. Eg: To receive TF data published over /tf_static
    // by various publishers.
    sub_options.query_consolidation = z_query_consolidation_none();
    if (sub_data->adapted_qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      sub_options.reliability = Z_RELIABILITY_RELIABLE;
    }
    sub_data->sub = ze_declare_querying_subscriber(
      z_loan(context_impl->session),
      z_loan(keyexpr),
      z_move(callback),
      &sub_options
    );
    if (!z_check(std::get<ze_owned_querying_subscriber_t>(sub_data->sub))) {
      RMW_SET_ERROR_MSG("unable to create zenoh subscription");
      return nullptr;
    }
  } else {
    // Create a regular subscriber for all other durability settings.
    z_subscriber_options_t sub_options = z_subscriber_options_default();
    if (qos_profile->reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      sub_options.reliability = Z_RELIABILITY_RELIABLE;
    }
    sub_data->sub = z_declare_subscriber(
      z_loan(context_impl->session),
      z_loan(keyexpr),
      z_move(callback),
      &sub_options
    );
    if (!z_check(std::get<z_owned_subscriber_t>(sub_data->sub))) {
      RMW_SET_ERROR_MSG("unable to create zenoh subscription");
      return nullptr;
    }
  }

  auto undeclare_z_sub = rcpputils::make_scope_exit(
    [sub_data]() {
      z_owned_subscriber_t * sub = std::get_if<z_owned_subscriber_t>(&sub_data->sub);
      if (sub == nullptr || z_undeclare_subscriber(sub)) {
        RMW_SET_ERROR_MSG("failed to undeclare sub");
      } else {
        ze_owned_querying_subscriber_t * querying_sub =
        std::get_if<ze_owned_querying_subscriber_t>(&sub_data->sub);
        if (querying_sub == nullptr || ze_undeclare_querying_subscriber(querying_sub)) {
          RMW_SET_ERROR_MSG("failed to undeclare sub");
        }
      }
    });

  // Publish to the graph that a new subscription is in town
  sub_data->entity = rmw_zenoh_cpp::liveliness::Entity::make(
    z_info_zid(z_loan(node->context->impl->session)),
    std::to_string(node_data->id),
    std::to_string(
      context_impl->get_next_entity_id()),
    rmw_zenoh_cpp::liveliness::EntityType::Subscription,
    rmw_zenoh_cpp::liveliness::NodeInfo{
      node->context->actual_domain_id, node->namespace_, node->name, context_impl->enclave},
    rmw_zenoh_cpp::liveliness::TopicInfo{
      rmw_subscription->topic_name,
      sub_data->type_support->get_name(),
      type_hash_c_str,
      sub_data->adapted_qos_profile}
  );
  if (sub_data->entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the subscription.");
    return nullptr;
  }
  sub_data->token = zc_liveliness_declare_token(
    z_loan(context_impl->session),
    z_keyexpr(sub_data->entity->keyexpr().c_str()),
    NULL
  );
  auto free_token = rcpputils::make_scope_exit(
    [sub_data]() {
      if (sub_data != nullptr) {
        z_drop(z_move(sub_data->token));
      }
    });
  if (!z_check(sub_data->token)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the subscription.");
    return nullptr;
  }

  rmw_subscription->data = sub_data;

  free_token.cancel();
  undeclare_z_sub.cancel();
  free_topic_name.cancel();
  destruct_msg_type_support.cancel();
  free_type_support.cancel();
  destruct_sub_data.cancel();
  free_sub_data.cancel();
  free_rmw_subscription.cancel();

  return rmw_subscription;
}

//==============================================================================
/// Finalize a given subscription handle, reclaim the resources, and deallocate the subscription
rmw_ret_t
rmw_destroy_subscription(rmw_node_t * node, rmw_subscription_t * subscription)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rmw_ret_t ret = RMW_RET_OK;

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  auto sub_data = static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(subscription->data);
  if (sub_data != nullptr) {
    // Publish to the graph that a subscription has ridden off into the sunset
    zc_liveliness_undeclare_token(z_move(sub_data->token));

    RMW_TRY_DESTRUCTOR(sub_data->type_support->~MessageTypeSupport(), MessageTypeSupport, );
    allocator->deallocate(sub_data->type_support, allocator->state);

    z_owned_subscriber_t * sub = std::get_if<z_owned_subscriber_t>(&sub_data->sub);
    if (sub != nullptr) {
      if (z_undeclare_subscriber(sub)) {
        RMW_SET_ERROR_MSG("failed to undeclare sub");
        ret = RMW_RET_ERROR;
      }
    } else {
      ze_owned_querying_subscriber_t * querying_sub =
        std::get_if<ze_owned_querying_subscriber_t>(&sub_data->sub);
      if (querying_sub == nullptr || ze_undeclare_querying_subscriber(querying_sub)) {
        RMW_SET_ERROR_MSG("failed to undeclare sub");
        ret = RMW_RET_ERROR;
      }
    }

    RMW_TRY_DESTRUCTOR(sub_data->~rmw_subscription_data_t(), rmw_subscription_data_t, );
    allocator->deallocate(sub_data, allocator->state);
  }
  allocator->deallocate(const_cast<char *>(subscription->topic_name), allocator->state);
  allocator->deallocate(subscription, allocator->state);

  return ret;
}

//==============================================================================
/// Retrieve the number of matched publishers to a subscription.
rmw_ret_t
rmw_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * publisher_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_count, RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_subscription_data_t * sub_data =
    static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data->context, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(sub_data->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);

  return context_impl->graph_cache->subscription_count_matched_publishers(
    subscription, publisher_count);
}

//==============================================================================
/// Retrieve the actual qos settings of the subscription.
rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  auto sub_data = static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  *qos = sub_data->adapted_qos_profile;
  return RMW_RET_OK;
}

//==============================================================================
/// Set the content filter options for the subscription.
rmw_ret_t
rmw_subscription_set_content_filter(
  rmw_subscription_t * subscription,
  const rmw_subscription_content_filter_options_t * options)
{
  static_cast<void>(subscription);
  static_cast<void>(options);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Retrieve the content filter options of the subscription.
rmw_ret_t
rmw_subscription_get_content_filter(
  const rmw_subscription_t * subscription,
  rcutils_allocator_t * allocator,
  rmw_subscription_content_filter_options_t * options)
{
  static_cast<void>(subscription);
  static_cast<void>(allocator);
  static_cast<void>(options);
  return RMW_RET_UNSUPPORTED;
}

namespace
{
rmw_ret_t
__rmw_take_one(
  rmw_zenoh_cpp::rmw_subscription_data_t * sub_data,
  void * ros_message,
  rmw_message_info_t * message_info,
  bool * taken)
{
  *taken = false;

  std::unique_ptr<rmw_zenoh_cpp::saved_msg_data> msg_data = sub_data->pop_next_message();
  if (msg_data == nullptr) {
    // There are no more messages to take.
    return RMW_RET_OK;
  }

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(const_cast<uint8_t *>(msg_data->payload.payload.start)),
    msg_data->payload.payload.len);

  // Object that serializes the data
  rmw_zenoh_cpp::Cdr deser(fastbuffer);
  if (!sub_data->type_support->deserialize_ros_message(
      deser.get_cdr(),
      ros_message,
      sub_data->type_support_impl))
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
}  // namespace

//==============================================================================
/// Take an incoming ROS message.
rmw_ret_t
rmw_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  *taken = false;

  auto sub_data = static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  return __rmw_take_one(sub_data, ros_message, nullptr, taken);
}

//==============================================================================
/// Take an incoming ROS message with its metadata.
rmw_ret_t
rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  *taken = false;

  auto sub_data = static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  return __rmw_take_one(sub_data, ros_message, message_info, taken);
}

//==============================================================================
/// Take multiple incoming ROS messages with their metadata.
rmw_ret_t
rmw_take_sequence(
  const rmw_subscription_t * subscription,
  size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  if (0u == count) {
    RMW_SET_ERROR_MSG("count cannot be 0");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > message_sequence->capacity) {
    RMW_SET_ERROR_MSG("Insuffient capacity in message_sequence");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > message_info_sequence->capacity) {
    RMW_SET_ERROR_MSG("Insuffient capacity in message_info_sequence");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > (std::numeric_limits<uint32_t>::max)()) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Cannot take %zu samples at once, limit is %" PRIu32,
      count, (std::numeric_limits<uint32_t>::max)());
    return RMW_RET_ERROR;
  }

  *taken = 0;

  auto sub_data = static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  if (sub_data->context->impl->is_shutdown) {
    return RMW_RET_OK;
  }

  rmw_ret_t ret;

  while (*taken < count) {
    bool one_taken = false;

    ret = __rmw_take_one(
      sub_data, message_sequence->data[*taken],
      &message_info_sequence->data[*taken], &one_taken);
    if (ret != RMW_RET_OK) {
      // If we are taking a sequence and the 2nd take in the sequence failed, we'll report
      // RMW_RET_ERROR to the caller, but we will *also* tell the caller that there are valid
      // messages already taken (via the message_sequence size).  It is up to the caller to deal
      // with that situation appropriately.
      break;
    }

    if (!one_taken) {
      // No error, but there was nothing left to be taken, so break out of the loop
      break;
    }

    (*taken)++;
  }

  message_sequence->size = *taken;
  message_info_sequence->size = *taken;

  return ret;
}

//==============================================================================
namespace
{
rmw_ret_t
__rmw_take_serialized(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  *taken = false;

  auto sub_data = static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  if (sub_data->context->impl->is_shutdown) {
    return RMW_RET_OK;
  }

  // RETRIEVE SERIALIZED MESSAGE ===============================================

  std::unique_ptr<rmw_zenoh_cpp::saved_msg_data> msg_data = sub_data->pop_next_message();
  if (msg_data == nullptr) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }

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
}  // namespace

//==============================================================================
/// Take an incoming ROS message as a byte stream.
rmw_ret_t
rmw_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);

  return __rmw_take_serialized(subscription, serialized_message, taken, nullptr);
}

//==============================================================================
/// Take an incoming ROS message as a byte stream with its metadata.
rmw_ret_t
rmw_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);

  return __rmw_take_serialized(subscription, serialized_message, taken, message_info);
}

//==============================================================================
/// Take an incoming ROS message, loaned by the middleware.
rmw_ret_t
rmw_take_loaned_message(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(subscription);
  static_cast<void>(loaned_message);
  static_cast<void>(taken);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Take a loaned message and with its additional message information.
rmw_ret_t
rmw_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(subscription);
  static_cast<void>(loaned_message);
  static_cast<void>(taken);
  static_cast<void>(message_info);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Return a loaned ROS message previously taken from a subscription.
rmw_ret_t
rmw_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message)
{
  static_cast<void>(subscription);
  static_cast<void>(loaned_message);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Create a service client that can send requests to and receive replies from a service server.
rmw_client_t *
rmw_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);

  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  if (strlen(service_name) == 0) {
    RMW_SET_ERROR_MSG("service name is empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);

  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return nullptr);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(
    node->context->impl);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl,
    "unable to get rmw_context_impl_s",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl->enclave,
    "expected initialized enclave",
    return nullptr);
  if (!z_check(context_impl->session)) {
    RMW_SET_ERROR_MSG("zenoh session is invalid");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(node->data, nullptr);
  const rmw_zenoh_cpp::rmw_node_data_t * node_data =
    static_cast<rmw_zenoh_cpp::rmw_node_data_t *>(node->data);
  if (node_data == nullptr) {
    RMW_SET_ERROR_MSG(
      "Unable to create client as node data is invalid.");
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // Validate service name
  int validation_result;

  if (rmw_validate_full_topic_name(service_name, &validation_result, nullptr) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("rmw_validate_full_topic_name failed");
    return nullptr;
  }

  if (validation_result != RMW_TOPIC_VALID && !qos_profile->avoid_ros_namespace_conventions) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("service name is malformed: %s", service_name);
    return nullptr;
  }

  // client data
  rmw_client_t * rmw_client = static_cast<rmw_client_t *>(allocator->zero_allocate(
      1,
      sizeof(rmw_client_t),
      allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_client,
    "failed to allocate memory for the client",
    return nullptr);

  auto free_rmw_client = rcpputils::make_scope_exit(
    [rmw_client, allocator]() {
      allocator->deallocate(rmw_client, allocator->state);
    });

  auto client_data = static_cast<rmw_zenoh_cpp::rmw_client_data_t *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::rmw_client_data_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client_data,
    "failed to allocate memory for client data",
    return nullptr);
  auto free_client_data = rcpputils::make_scope_exit(
    [client_data, allocator]() {
      allocator->deallocate(client_data, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(client_data, client_data, return nullptr, rmw_zenoh_cpp::rmw_client_data_t);
  auto destruct_client_data = rcpputils::make_scope_exit(
    [client_data]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        client_data->~rmw_client_data_t(),
        rmw_zenoh_cpp::rmw_client_data_t);
    });

  generate_random_gid(client_data->client_gid);

  // Adapt any 'best available' QoS options
  client_data->adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = rmw_zenoh_cpp::QoS::get().best_available_qos(
    nullptr, nullptr, &client_data->adapted_qos_profile, nullptr);
  if (RMW_RET_OK != ret) {
    RMW_SET_ERROR_MSG("Failed to obtain adapted_qos_profile.");
    return nullptr;
  }

  // Obtain the type support
  const rosidl_service_type_support_t * type_support = find_service_type_support(type_supports);
  if (type_support == nullptr) {
    // error was already set by find_service_type_support
    return nullptr;
  }

  auto service_members = static_cast<const service_type_support_callbacks_t *>(type_support->data);
  auto request_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->request_members_->data);
  auto response_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->response_members_->data);

  client_data->context = node->context;
  client_data->typesupport_identifier = type_support->typesupport_identifier;
  client_data->type_hash = type_support->get_type_hash_func(type_support);
  client_data->request_type_support_impl = request_members;
  client_data->response_type_support_impl = response_members;

  // Request type support
  client_data->request_type_support = static_cast<rmw_zenoh_cpp::RequestTypeSupport *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::RequestTypeSupport), allocator->state));

  RMW_CHECK_FOR_NULL_WITH_MSG(
    client_data->request_type_support,
    "Failed to allocate rmw_zenoh_cpp::RequestTypeSupport",
    return nullptr);
  auto free_request_type_support = rcpputils::make_scope_exit(
    [client_data, allocator]() {
      allocator->deallocate(client_data->request_type_support, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(
    client_data->request_type_support,
    client_data->request_type_support,
    return nullptr,
    rmw_zenoh_cpp::RequestTypeSupport, service_members);
  auto destruct_request_type_support = rcpputils::make_scope_exit(
    [client_data]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        client_data->request_type_support->~RequestTypeSupport(),
        rmw_zenoh_cpp::RequestTypeSupport);
    });

  // Response type support
  client_data->response_type_support = static_cast<rmw_zenoh_cpp::ResponseTypeSupport *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::ResponseTypeSupport), allocator->state));

  RMW_CHECK_FOR_NULL_WITH_MSG(
    client_data->response_type_support,
    "Failed to allocate rmw_zenoh_cpp::ResponseTypeSupport",
    return nullptr);
  auto free_response_type_support = rcpputils::make_scope_exit(
    [client_data, allocator]() {
      allocator->deallocate(client_data->response_type_support, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(
    client_data->response_type_support,
    client_data->response_type_support,
    return nullptr,
    rmw_zenoh_cpp::ResponseTypeSupport, service_members);
  auto destruct_response_type_support = rcpputils::make_scope_exit(
    [client_data]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        client_data->response_type_support->~ResponseTypeSupport(),
        rmw_zenoh_cpp::ResponseTypeSupport);
    });

  // Populate the rmw_client.
  rmw_client->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  rmw_client->service_name = rcutils_strdup(service_name, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_client->service_name,
    "failed to allocate client name",
    return nullptr);
  auto free_service_name = rcpputils::make_scope_exit(
    [rmw_client, allocator]() {
      allocator->deallocate(const_cast<char *>(rmw_client->service_name), allocator->state);
    });

  // Note: Service request/response types will contain a suffix Request_ or Response_.
  // We remove the suffix when appending the type to the liveliness tokens for
  // better reusability within GraphCache.
  std::string service_type = client_data->request_type_support->get_name();
  size_t suffix_substring_position = service_type.find("Request_");
  if (std::string::npos != suffix_substring_position) {
    service_type = service_type.substr(0, suffix_substring_position);
  } else {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unexpected type %s for client %s. Report this bug",
      service_type.c_str(), rmw_client->service_name);
    return nullptr;
  }

  // Convert the type hash to a string so that it can be included in
  // the keyexpr.
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(
    client_data->type_hash,
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

  client_data->keyexpr = ros_topic_name_to_zenoh_key(
    node->context->actual_domain_id,
    rmw_client->service_name,
    service_type.c_str(),
    type_hash_c_str);
  auto free_ros_keyexpr = rcpputils::make_scope_exit(
    [client_data]() {
      z_keyexpr_drop(z_move(client_data->keyexpr));
    });
  if (!z_keyexpr_check(&client_data->keyexpr)) {
    RMW_SET_ERROR_MSG("unable to create zenoh keyexpr.");
    return nullptr;
  }

  client_data->entity = rmw_zenoh_cpp::liveliness::Entity::make(
    z_info_zid(z_loan(node->context->impl->session)),
    std::to_string(node_data->id),
    std::to_string(
      context_impl->get_next_entity_id()),
    rmw_zenoh_cpp::liveliness::EntityType::Client,
    rmw_zenoh_cpp::liveliness::NodeInfo{
      node->context->actual_domain_id, node->namespace_, node->name, context_impl->enclave},
    rmw_zenoh_cpp::liveliness::TopicInfo{
      rmw_client->service_name,
      std::move(service_type),
      type_hash_c_str,
      client_data->adapted_qos_profile}
  );
  if (client_data->entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the client.");
    return nullptr;
  }
  client_data->token = zc_liveliness_declare_token(
    z_loan(node->context->impl->session),
    z_keyexpr(client_data->entity->keyexpr().c_str()),
    NULL
  );
  auto free_token = rcpputils::make_scope_exit(
    [client_data]() {
      if (client_data != nullptr) {
        z_drop(z_move(client_data->token));
      }
    });
  if (!z_check(client_data->token)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the client.");
    return nullptr;
  }

  rmw_client->data = client_data;

  free_token.cancel();
  free_rmw_client.cancel();
  free_client_data.cancel();
  destruct_request_type_support.cancel();
  free_request_type_support.cancel();
  destruct_response_type_support.cancel();
  free_response_type_support.cancel();
  destruct_client_data.cancel();
  free_service_name.cancel();
  free_ros_keyexpr.cancel();

  return rmw_client;
}

//==============================================================================
/// Destroy and unregister a service client from its node.
rmw_ret_t
rmw_destroy_client(rmw_node_t * node, rmw_client_t * client)
{
  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  rmw_zenoh_cpp::rmw_client_data_t * client_data =
    static_cast<rmw_zenoh_cpp::rmw_client_data_t *>(client->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client_data,
    "client implementation pointer is null.",
    return RMW_RET_INVALID_ARGUMENT);

  // CLEANUP ===================================================================
  z_drop(z_move(client_data->keyexpr));
  zc_liveliness_undeclare_token(z_move(client_data->token));

  RMW_TRY_DESTRUCTOR(
    client_data->request_type_support->~RequestTypeSupport(), rmw_zenoh_cpp::RequestTypeSupport, );
  allocator->deallocate(client_data->request_type_support, allocator->state);

  RMW_TRY_DESTRUCTOR(
    client_data->response_type_support->~ResponseTypeSupport(), rmw_zenoh_cpp::ResponseTypeSupport,
  );
  allocator->deallocate(client_data->response_type_support, allocator->state);

  // See the comment about the "num_in_flight" class variable in the rmw_client_data_t class for
  // why we need to do this.
  if (!client_data->shutdown_and_query_in_flight()) {
    RMW_TRY_DESTRUCTOR(client_data->~rmw_client_data_t(), rmw_client_data_t, );
    allocator->deallocate(client->data, allocator->state);
  }

  allocator->deallocate(const_cast<char *>(client->service_name), allocator->state);
  allocator->deallocate(client, allocator->state);

  return RMW_RET_OK;
}

//==============================================================================
/// Send a ROS service request.
rmw_ret_t
rmw_send_request(
  const rmw_client_t * client,
  const void * ros_request,
  int64_t * sequence_id)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(sequence_id, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rmw_zenoh_cpp::rmw_client_data_t * client_data =
    static_cast<rmw_zenoh_cpp::rmw_client_data_t *>(client->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client_data,
    "Unable to retrieve client_data from client.",
    RMW_RET_INVALID_ARGUMENT);

  if (client_data->is_shutdown()) {
    return RMW_RET_ERROR;
  }

  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(
    client_data->context->impl);

  // Serialize data

  rcutils_allocator_t * allocator = &(client_data->context->options.allocator);

  size_t max_data_length = (
    client_data->request_type_support->get_estimated_serialized_size(
      ros_request, client_data->request_type_support_impl));

  // Init serialized message byte array
  char * request_bytes = static_cast<char *>(allocator->allocate(
      max_data_length,
      allocator->state));
  if (!request_bytes) {
    RMW_SET_ERROR_MSG("failed allocate request message bytes");
    return RMW_RET_ERROR;
  }
  auto free_request_bytes = rcpputils::make_scope_exit(
    [request_bytes, allocator]() {
      allocator->deallocate(request_bytes, allocator->state);
    });

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(request_bytes, max_data_length);

  // Object that serializes the data
  rmw_zenoh_cpp::Cdr ser(fastbuffer);
  if (!client_data->request_type_support->serialize_ros_message(
      ros_request,
      ser.get_cdr(),
      client_data->request_type_support_impl))
  {
    return RMW_RET_ERROR;
  }

  size_t data_length = ser.get_serialized_data_length();

  *sequence_id = client_data->get_next_sequence_number();

  // Send request
  z_get_options_t opts = z_get_options_default();

  z_owned_bytes_map_t map = create_map_and_set_sequence_num(*sequence_id, client_data->client_gid);
  if (!z_check(map)) {
    // create_map_and_set_sequence_num already set the error
    return RMW_RET_ERROR;
  }
  auto free_attachment_map = rcpputils::make_scope_exit(
    [&map]() {
      z_bytes_map_drop(z_move(map));
    });

  // See the comment about the "num_in_flight" class variable in the rmw_client_data_t class for
  // why we need to do this.
  client_data->increment_in_flight_callbacks();

  opts.attachment = z_bytes_map_as_attachment(&map);

  opts.target = Z_QUERY_TARGET_ALL_COMPLETE;
  // The default timeout for a z_get query is 10 seconds and if a response is not received within
  // this window, the queryable will return an invalid reply. However, it is common for actions,
  // which are implemented using services, to take an extended duration to complete. Hence, we set
  // the timeout_ms to the largest supported value to account for most realistic scenarios.
  opts.timeout_ms = std::numeric_limits<uint64_t>::max();
  // Latest consolidation guarantees unicity of replies for the same key expression,
  // which optimizes bandwidth. The default is "None", which imples replies may come in any order
  // and any number.
  opts.consolidation = z_query_consolidation_latest();
  opts.value.payload = z_bytes_t{data_length, reinterpret_cast<const uint8_t *>(request_bytes)};
  // z_owned_closure_reply_t zn_closure_reply =
  //   z_closure(rmw_zenoh_cpp::client_data_handler, rmw_zenoh_cpp::client_data_drop, client_data);
  z_owned_closure_reply_t zn_closure_reply;
  zn_closure_reply.context = (void *)client_data;
  zn_closure_reply.call = rmw_zenoh_cpp::client_data_handler;
  zn_closure_reply.drop = rmw_zenoh_cpp::client_data_drop;
  z_get(
    z_loan(context_impl->session),
    z_loan(client_data->keyexpr), "",
    z_move(zn_closure_reply),
    &opts);

  return RMW_RET_OK;
}

//==============================================================================
/// Take an incoming ROS service response.
rmw_ret_t
rmw_take_response(
  const rmw_client_t * client,
  rmw_service_info_t * request_header,
  void * ros_response,
  bool * taken)
{
  *taken = false;

  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client->service_name, "client has no service name", RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_client_data_t * client_data =
    static_cast<rmw_zenoh_cpp::rmw_client_data_t *>(client->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client->data, "Unable to retrieve client_data from client.", RMW_RET_INVALID_ARGUMENT);

  std::unique_ptr<rmw_zenoh_cpp::ZenohReply> latest_reply = client_data->pop_next_reply();
  if (latest_reply == nullptr) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }

  std::optional<z_sample_t> sample = latest_reply->get_sample();
  if (!sample) {
    RMW_SET_ERROR_MSG("invalid reply sample");
    return RMW_RET_ERROR;
  }

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(const_cast<uint8_t *>(sample->payload.start)),
    sample->payload.len);

  // Object that serializes the data
  rmw_zenoh_cpp::Cdr deser(fastbuffer);
  if (!client_data->response_type_support->deserialize_ros_message(
      deser.get_cdr(),
      ros_response,
      client_data->response_type_support_impl))
  {
    RMW_SET_ERROR_MSG("could not deserialize ROS response");
    return RMW_RET_ERROR;
  }

  // Fill in the request_header

  request_header->request_id.sequence_number =
    rmw_zenoh_cpp::get_int64_from_attachment(&sample->attachment, "sequence_number");
  if (request_header->request_id.sequence_number < 0) {
    RMW_SET_ERROR_MSG("Failed to get sequence_number from client call attachment");
    return RMW_RET_ERROR;
  }

  request_header->source_timestamp =
    rmw_zenoh_cpp::get_int64_from_attachment(&sample->attachment, "source_timestamp");
  if (request_header->source_timestamp < 0) {
    RMW_SET_ERROR_MSG("Failed to get source_timestamp from client call attachment");
    return RMW_RET_ERROR;
  }

  if (!rmw_zenoh_cpp::get_gid_from_attachment(
      &sample->attachment,
      request_header->request_id.writer_guid))
  {
    RMW_SET_ERROR_MSG("Could not get client gid from attachment");
    return RMW_RET_ERROR;
  }

  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  request_header->received_timestamp = now_ns.count();

  *taken = true;

  return RMW_RET_OK;
}

//==============================================================================
/// Retrieve the actual qos settings of the client's request publisher.
rmw_ret_t
rmw_client_request_publisher_get_actual_qos(
  const rmw_client_t * client,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_client_data_t * client_data =
    static_cast<rmw_zenoh_cpp::rmw_client_data_t *>(client->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);

  *qos = client_data->adapted_qos_profile;
  return RMW_RET_OK;
}

//==============================================================================
/// Retrieve the actual qos settings of the client's response subscription.
rmw_ret_t
rmw_client_response_subscription_get_actual_qos(
  const rmw_client_t * client,
  rmw_qos_profile_t * qos)
{
  // The same QoS profile is used for sending requests and receiving responses.
  return rmw_client_request_publisher_get_actual_qos(client, qos);
}

//==============================================================================
/// Create a service server that can receive requests from and send replies to a service client.
rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  if (0 == strlen(service_name)) {
    RMW_SET_ERROR_MSG("service_name argument is an empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  if (!qos_profile->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    // TODO(francocipollone): Verify if this is the right way to validate the service name.
    rmw_ret_t ret = rmw_validate_full_topic_name(service_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("service_name argument is invalid: %s", reason);
      return nullptr;
    }
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(node->data, nullptr);
  const rmw_zenoh_cpp::rmw_node_data_t * node_data =
    static_cast<rmw_zenoh_cpp::rmw_node_data_t *>(node->data);
  if (node_data == nullptr) {
    RMW_SET_ERROR_MSG(
      "Unable to create service as node data is invalid.");
    return nullptr;
  }
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return nullptr);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(
    node->context->impl);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl,
    "unable to get rmw_context_impl_s",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl->enclave,
    "expected initialized enclave",
    return nullptr);
  if (!z_check(context_impl->session)) {
    RMW_SET_ERROR_MSG("zenoh session is invalid");
    return nullptr;
  }

  // SERVICE DATA ==============================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  rmw_service_t * rmw_service = static_cast<rmw_service_t *>(allocator->zero_allocate(
      1,
      sizeof(rmw_service_t),
      allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_service,
    "failed to allocate memory for the service",
    return nullptr);
  auto free_rmw_service = rcpputils::make_scope_exit(
    [rmw_service, allocator]() {
      allocator->deallocate(rmw_service, allocator->state);
    });

  auto service_data = static_cast<rmw_zenoh_cpp::rmw_service_data_t *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::rmw_service_data_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    service_data,
    "failed to allocate memory for service data",
    return nullptr);
  auto free_service_data = rcpputils::make_scope_exit(
    [service_data, allocator]() {
      allocator->deallocate(service_data, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(
    service_data, service_data, return nullptr,
    rmw_zenoh_cpp::rmw_service_data_t);
  auto destruct_service_data = rcpputils::make_scope_exit(
    [service_data]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        service_data->~rmw_service_data_t(),
        rmw_zenoh_cpp::rmw_service_data_t);
    });

  // Adapt any 'best available' QoS options
  service_data->adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = rmw_zenoh_cpp::QoS::get().best_available_qos(
    nullptr, nullptr, &service_data->adapted_qos_profile, nullptr);
  if (RMW_RET_OK != ret) {
    RMW_SET_ERROR_MSG("Failed to obtain adapted_qos_profile.");
    return nullptr;
  }

  // Get the RMW type support.
  const rosidl_service_type_support_t * type_support = find_service_type_support(type_supports);
  if (type_support == nullptr) {
    // error was already set by find_service_type_support
    return nullptr;
  }

  auto service_members = static_cast<const service_type_support_callbacks_t *>(type_support->data);
  auto request_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->request_members_->data);
  auto response_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->response_members_->data);

  service_data->context = node->context;
  service_data->typesupport_identifier = type_support->typesupport_identifier;
  service_data->type_hash = type_support->get_type_hash_func(type_support);
  service_data->request_type_support_impl = request_members;
  service_data->response_type_support_impl = response_members;

  // Request type support
  service_data->request_type_support = static_cast<rmw_zenoh_cpp::RequestTypeSupport *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::RequestTypeSupport), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    service_data->request_type_support,
    "Failed to allocate rmw_zenoh_cpp::RequestTypeSupport",
    return nullptr);
  auto free_request_type_support = rcpputils::make_scope_exit(
    [request_type_support = service_data->request_type_support, allocator]() {
      allocator->deallocate(request_type_support, allocator->state);
    });
  RMW_TRY_PLACEMENT_NEW(
    service_data->request_type_support,
    service_data->request_type_support,
    return nullptr,
    rmw_zenoh_cpp::RequestTypeSupport, service_members);
  auto destruct_request_type_support = rcpputils::make_scope_exit(
    [service_data]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        service_data->request_type_support->~RequestTypeSupport(),
        rmw_zenoh_cpp::RequestTypeSupport);
    });

  // Response type support
  service_data->response_type_support = static_cast<rmw_zenoh_cpp::ResponseTypeSupport *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::ResponseTypeSupport), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    service_data->response_type_support,
    "Failed to allocate rmw_zenoh_cpp::ResponseTypeSupport",
    return nullptr);
  auto free_response_type_support = rcpputils::make_scope_exit(
    [response_type_support = service_data->response_type_support, allocator]() {
      allocator->deallocate(response_type_support, allocator->state);
    });
  RMW_TRY_PLACEMENT_NEW(
    service_data->response_type_support,
    service_data->response_type_support,
    return nullptr,
    rmw_zenoh_cpp::ResponseTypeSupport, service_members);
  auto destruct_response_type_support = rcpputils::make_scope_exit(
    [service_data]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        service_data->response_type_support->~ResponseTypeSupport(),
        rmw_zenoh_cpp::ResponseTypeSupport);
    });

  // Populate the rmw_service.
  rmw_service->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  rmw_service->service_name = rcutils_strdup(service_name, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_service->service_name,
    "failed to allocate service name",
    return nullptr);
  auto free_service_name = rcpputils::make_scope_exit(
    [rmw_service, allocator]() {
      allocator->deallocate(const_cast<char *>(rmw_service->service_name), allocator->state);
    });

  // Note: Service request/response types will contain a suffix Request_ or Response_.
  // We remove the suffix when appending the type to the liveliness tokens for
  // better reusability within GraphCache.
  std::string service_type = service_data->response_type_support->get_name();
  size_t suffix_substring_position = service_type.find("Response_");
  if (std::string::npos != suffix_substring_position) {
    service_type = service_type.substr(0, suffix_substring_position);
  } else {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unexpected type %s for service %s. Report this bug",
      service_type.c_str(), rmw_service->service_name);
    return nullptr;
  }

  // Convert the type hash to a string so that it can be included in
  // the keyexpr.
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(
    service_data->type_hash,
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

  service_data->keyexpr = ros_topic_name_to_zenoh_key(
    node->context->actual_domain_id,
    rmw_service->service_name,
    service_type.c_str(),
    type_hash_c_str);
  auto free_ros_keyexpr = rcpputils::make_scope_exit(
    [service_data]() {
      if (service_data) {
        z_drop(z_move(service_data->keyexpr));
      }
    });
  if (!z_check(z_loan(service_data->keyexpr))) {
    RMW_SET_ERROR_MSG("unable to create zenoh keyexpr.");
    return nullptr;
  }

  // z_owned_closure_query_t callback = z_closure(
  //   rmw_zenoh_cpp::service_data_handler, nullptr,
  //   service_data);
  z_owned_closure_query_t callback;
  callback.context = (void *)service_data;
  callback.call = rmw_zenoh_cpp::service_data_handler;
  callback.drop = nullptr;
  // Configure the queryable to process complete queries.
  z_queryable_options_t qable_options = z_queryable_options_default();
  qable_options.complete = true;
  service_data->qable = z_declare_queryable(
    z_loan(context_impl->session),
    z_loan(service_data->keyexpr),
    z_move(callback),
    &qable_options);

  if (!z_check(service_data->qable)) {
    RMW_SET_ERROR_MSG("unable to create zenoh queryable");
    return nullptr;
  }
  auto undeclare_z_queryable = rcpputils::make_scope_exit(
    [service_data]() {
      z_undeclare_queryable(z_move(service_data->qable));
    });

  service_data->entity = rmw_zenoh_cpp::liveliness::Entity::make(
    z_info_zid(z_loan(node->context->impl->session)),
    std::to_string(node_data->id),
    std::to_string(
      context_impl->get_next_entity_id()),
    rmw_zenoh_cpp::liveliness::EntityType::Service,
    rmw_zenoh_cpp::liveliness::NodeInfo{
      node->context->actual_domain_id, node->namespace_, node->name, context_impl->enclave},
    rmw_zenoh_cpp::liveliness::TopicInfo{
      rmw_service->service_name,
      std::move(service_type),
      type_hash_c_str,
      service_data->adapted_qos_profile}
  );
  if (service_data->entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the service.");
    return nullptr;
  }
  service_data->token = zc_liveliness_declare_token(
    z_loan(node->context->impl->session),
    z_keyexpr(service_data->entity->keyexpr().c_str()),
    NULL
  );
  auto free_token = rcpputils::make_scope_exit(
    [service_data]() {
      if (service_data != nullptr) {
        z_drop(z_move(service_data->token));
      }
    });
  if (!z_check(service_data->token)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the service.");
    return nullptr;
  }

  rmw_service->data = service_data;

  free_rmw_service.cancel();
  free_service_data.cancel();
  free_service_name.cancel();
  destruct_service_data.cancel();
  destruct_request_type_support.cancel();
  destruct_response_type_support.cancel();
  free_request_type_support.cancel();
  free_response_type_support.cancel();
  free_ros_keyexpr.cancel();
  undeclare_z_queryable.cancel();
  free_token.cancel();

  return rmw_service;
}

//==============================================================================
/// Destroy and unregister a service server from its node.
rmw_ret_t
rmw_destroy_service(rmw_node_t * node, rmw_service_t * service)
{
  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  rmw_zenoh_cpp::rmw_service_data_t * service_data =
    static_cast<rmw_zenoh_cpp::rmw_service_data_t *>(service->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    service_data,
    "Unable to retrieve service_data from service",
    return RMW_RET_INVALID_ARGUMENT);

  // CLEANUP ================================================================
  z_drop(z_move(service_data->keyexpr));
  z_undeclare_queryable(z_move(service_data->qable));
  zc_liveliness_undeclare_token(z_move(service_data->token));

  RMW_TRY_DESTRUCTOR(
    service_data->request_type_support->~RequestTypeSupport(), rmw_zenoh_cpp::RequestTypeSupport, );
  allocator->deallocate(service_data->request_type_support, allocator->state);

  RMW_TRY_DESTRUCTOR(
    service_data->response_type_support->~ResponseTypeSupport(), rmw_zenoh_cpp::ResponseTypeSupport,
  );
  allocator->deallocate(service_data->response_type_support, allocator->state);

  RMW_TRY_DESTRUCTOR(service_data->~rmw_service_data_t(), rmw_zenoh_cpp::rmw_service_data_t, );
  allocator->deallocate(service->data, allocator->state);

  allocator->deallocate(const_cast<char *>(service->service_name), allocator->state);
  allocator->deallocate(service, allocator->state);

  return RMW_RET_OK;
}

//==============================================================================
/// Take an incoming ROS service request.
rmw_ret_t
rmw_take_request(
  const rmw_service_t * service,
  rmw_service_info_t * request_header,
  void * ros_request,
  bool * taken)
{
  *taken = false;

  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_FOR_NULL_WITH_MSG(
    service->service_name, "service has no service name", RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_service_data_t * service_data =
    static_cast<rmw_zenoh_cpp::rmw_service_data_t *>(service->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    service->data, "Unable to retrieve service_data from service", RMW_RET_INVALID_ARGUMENT);

  std::unique_ptr<rmw_zenoh_cpp::ZenohQuery> query = service_data->pop_next_query();
  if (query == nullptr) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }

  const z_query_t loaned_query = query->get_query();

  // DESERIALIZE MESSAGE ========================================================
  z_value_t payload_value = z_query_value(&loaned_query);

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(const_cast<uint8_t *>(payload_value.payload.start)),
    payload_value.payload.len);

  // Object that serializes the data
  rmw_zenoh_cpp::Cdr deser(fastbuffer);
  if (!service_data->request_type_support->deserialize_ros_message(
      deser.get_cdr(),
      ros_request,
      service_data->request_type_support_impl))
  {
    RMW_SET_ERROR_MSG("could not deserialize ROS message");
    return RMW_RET_ERROR;
  }

  // Fill in the request header.

  // Get the sequence_number out of the attachment
  z_attachment_t attachment = z_query_attachment(&loaned_query);

  request_header->request_id.sequence_number =
    rmw_zenoh_cpp::get_int64_from_attachment(&attachment, "sequence_number");
  if (request_header->request_id.sequence_number < 0) {
    RMW_SET_ERROR_MSG("Failed to get sequence_number from client call attachment");
    return RMW_RET_ERROR;
  }

  request_header->source_timestamp = rmw_zenoh_cpp::get_int64_from_attachment(
    &attachment,
    "source_timestamp");
  if (request_header->source_timestamp < 0) {
    RMW_SET_ERROR_MSG("Failed to get source_timestamp from client call attachment");
    return RMW_RET_ERROR;
  }

  if (!rmw_zenoh_cpp::get_gid_from_attachment(
      &attachment,
      request_header->request_id.writer_guid))
  {
    RMW_SET_ERROR_MSG("Could not get client GID from attachment");
    return RMW_RET_ERROR;
  }

  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  request_header->received_timestamp = now_ns.count();

  // Add this query to the map, so that rmw_send_response can quickly look it up later
  if (!service_data->add_to_query_map(request_header->request_id, std::move(query))) {
    RMW_SET_ERROR_MSG("duplicate sequence number in the map");
    return RMW_RET_ERROR;
  }

  *taken = true;

  return RMW_RET_OK;
}

//==============================================================================
/// Send a ROS service response.
rmw_ret_t
rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header,
  void * ros_response)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_FOR_NULL_WITH_MSG(
    service->data,
    "Unable to retrieve service_data from service",
    RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_service_data_t * service_data =
    static_cast<rmw_zenoh_cpp::rmw_service_data_t *>(service->data);

  // Create the queryable payload
  std::unique_ptr<rmw_zenoh_cpp::ZenohQuery> query =
    service_data->take_from_query_map(*request_header);
  if (query == nullptr) {
    // If there is no data associated with this request, the higher layers of
    // ROS 2 seem to expect that we just silently return with no work.
    return RMW_RET_OK;
  }

  rcutils_allocator_t * allocator = &(service_data->context->options.allocator);

  size_t max_data_length = (
    service_data->response_type_support->get_estimated_serialized_size(
      ros_response, service_data->response_type_support_impl));

  // Init serialized message byte array
  char * response_bytes = static_cast<char *>(allocator->allocate(
      max_data_length,
      allocator->state));
  if (!response_bytes) {
    RMW_SET_ERROR_MSG("failed to allocate response message bytes");
    return RMW_RET_ERROR;
  }
  auto free_response_bytes = rcpputils::make_scope_exit(
    [response_bytes, allocator]() {
      allocator->deallocate(response_bytes, allocator->state);
    });

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(response_bytes, max_data_length);

  // Object that serializes the data
  rmw_zenoh_cpp::Cdr ser(fastbuffer);
  if (!service_data->response_type_support->serialize_ros_message(
      ros_response,
      ser.get_cdr(),
      service_data->response_type_support_impl))
  {
    return RMW_RET_ERROR;
  }

  size_t data_length = ser.get_serialized_data_length();

  const z_query_t loaned_query = query->get_query();
  z_query_reply_options_t options = z_query_reply_options_default();

  z_owned_bytes_map_t map = create_map_and_set_sequence_num(
    request_header->sequence_number, request_header->writer_guid);
  if (!z_check(map)) {
    // create_map_and_set_sequence_num already set the error
    return RMW_RET_ERROR;
  }
  auto free_attachment_map = rcpputils::make_scope_exit(
    [&map]() {
      z_bytes_map_drop(z_move(map));
    });

  options.attachment = z_bytes_map_as_attachment(&map);

  z_query_reply(
    &loaned_query, z_loan(service_data->keyexpr), reinterpret_cast<const uint8_t *>(
      response_bytes), data_length, &options);

  return RMW_RET_OK;
}

//==============================================================================
/// Retrieve the actual qos settings of the service's request subscription.
rmw_ret_t
rmw_service_request_subscription_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_service_data_t * service_data =
    static_cast<rmw_zenoh_cpp::rmw_service_data_t *>(service->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);

  *qos = service_data->adapted_qos_profile;
  return RMW_RET_OK;
}

//==============================================================================
/// Retrieve the actual qos settings of the service's response publisher.
rmw_ret_t
rmw_service_response_publisher_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  // The same QoS profile is used for receiving requests and sending responses.
  return rmw_service_request_subscription_get_actual_qos(service, qos);
}

//==============================================================================
/// Create a guard condition and return a handle to that guard condition.
rmw_guard_condition_t *
rmw_create_guard_condition(rmw_context_t * context)
{
  rcutils_allocator_t * allocator = &context->options.allocator;

  auto guard_condition =
    static_cast<rmw_guard_condition_t *>(allocator->zero_allocate(
      1, sizeof(rmw_guard_condition_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    guard_condition,
    "unable to allocate memory for guard_condition",
    return nullptr);
  auto free_guard_condition = rcpputils::make_scope_exit(
    [guard_condition, allocator]() {
      allocator->deallocate(guard_condition, allocator->state);
    });

  guard_condition->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  guard_condition->context = context;

  guard_condition->data = allocator->zero_allocate(
    1, sizeof(rmw_zenoh_cpp::GuardCondition),
    allocator->state);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    guard_condition->data,
    "unable to allocate memory for guard condition data",
    return nullptr);
  auto free_guard_condition_data = rcpputils::make_scope_exit(
    [guard_condition, allocator]() {
      allocator->deallocate(guard_condition->data, allocator->state);
    });

  new(guard_condition->data) rmw_zenoh_cpp::GuardCondition;
  auto destruct_guard_condition = rcpputils::make_scope_exit(
    [guard_condition]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        static_cast<rmw_zenoh_cpp::GuardCondition *>(guard_condition->data)->~GuardCondition(),
        rmw_zenoh_cpp::GuardCondition);
    });

  destruct_guard_condition.cancel();
  free_guard_condition_data.cancel();
  free_guard_condition.cancel();

  return guard_condition;
}

/// Finalize a given guard condition handle, reclaim the resources, and deallocate the handle.
rmw_ret_t
rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);

  rcutils_allocator_t * allocator = &guard_condition->context->options.allocator;

  if (guard_condition->data) {
    static_cast<rmw_zenoh_cpp::GuardCondition *>(guard_condition->data)->~GuardCondition();
    allocator->deallocate(guard_condition->data, allocator->state);
  }

  allocator->deallocate(guard_condition, allocator->state);

  return RMW_RET_OK;
}

//==============================================================================
rmw_ret_t
rmw_trigger_guard_condition(const rmw_guard_condition_t * guard_condition)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    guard_condition,
    guard_condition->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  static_cast<rmw_zenoh_cpp::GuardCondition *>(guard_condition->data)->trigger();

  return RMW_RET_OK;
}

//==============================================================================
/// Create a wait set to store conditions that the middleware can wait on.
rmw_wait_set_t *
rmw_create_wait_set(rmw_context_t * context, size_t max_conditions)
{
  static_cast<void>(max_conditions);

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);

  rcutils_allocator_t * allocator = &context->options.allocator;

  auto wait_set = static_cast<rmw_wait_set_t *>(
    allocator->zero_allocate(1, sizeof(rmw_wait_set_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    wait_set,
    "failed to allocate wait set",
    return nullptr);
  auto cleanup_wait_set = rcpputils::make_scope_exit(
    [wait_set, allocator]() {
      allocator->deallocate(wait_set, allocator->state);
    });

  wait_set->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;

  wait_set->data = allocator->zero_allocate(
    1, sizeof(rmw_zenoh_cpp::rmw_wait_set_data_t),
    allocator->state);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    wait_set->data,
    "failed to allocate wait set data",
    return nullptr);
  auto free_wait_set_data = rcpputils::make_scope_exit(
    [wait_set, allocator]() {
      allocator->deallocate(wait_set->data, allocator->state);
    });

  // Invoke placement new
  new(wait_set->data) rmw_zenoh_cpp::rmw_wait_set_data_t;
  auto destruct_rmw_wait_set_data = rcpputils::make_scope_exit(
    [wait_set]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        static_cast<rmw_zenoh_cpp::rmw_wait_set_data_t *>(wait_set->data)->~rmw_wait_set_data_t(),
        rmw_wait_set_data);
    });

  static_cast<rmw_zenoh_cpp::rmw_wait_set_data_t *>(wait_set->data)->context = context;

  destruct_rmw_wait_set_data.cancel();
  free_wait_set_data.cancel();
  cleanup_wait_set.cancel();

  return wait_set;
}

//==============================================================================
/// Destroy a wait set.
rmw_ret_t
rmw_destroy_wait_set(rmw_wait_set_t * wait_set)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    wait_set,
    wait_set->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto wait_set_data = static_cast<rmw_zenoh_cpp::rmw_wait_set_data_t *>(wait_set->data);

  rcutils_allocator_t * allocator = &wait_set_data->context->options.allocator;

  wait_set_data->~rmw_wait_set_data_t();
  allocator->deallocate(wait_set_data, allocator->state);

  allocator->deallocate(wait_set, allocator->state);

  return RMW_RET_OK;
}

namespace
{
bool
check_and_attach_condition(
  const rmw_subscriptions_t * const subscriptions,
  const rmw_guard_conditions_t * const guard_conditions,
  const rmw_services_t * const services,
  const rmw_clients_t * const clients,
  const rmw_events_t * const events,
  rmw_zenoh_cpp::rmw_wait_set_data_t * wait_set_data)
{
  if (guard_conditions) {
    for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
      rmw_zenoh_cpp::GuardCondition * gc =
        static_cast<rmw_zenoh_cpp::GuardCondition *>(guard_conditions->guard_conditions[i]);
      if (gc == nullptr) {
        continue;
      }
      if (gc->check_and_attach_condition_if_not(wait_set_data)) {
        return true;
      }
    }
  }

  if (events) {
    for (size_t i = 0; i < events->event_count; ++i) {
      auto event = static_cast<rmw_event_t *>(events->events[i]);
      rmw_zenoh_cpp::rmw_zenoh_event_type_t zenoh_event_type =
        rmw_zenoh_cpp::zenoh_event_from_rmw_event(event->event_type);
      if (zenoh_event_type == rmw_zenoh_cpp::ZENOH_EVENT_INVALID) {
        RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
          "has_triggered_condition() called with unknown event %u. Report this bug.",
          event->event_type);
        continue;
      }

      auto event_data = static_cast<rmw_zenoh_cpp::EventsManager *>(event->data);
      if (event_data == nullptr) {
        continue;
      }

      if (event_data->queue_has_data_and_attach_condition_if_not(zenoh_event_type, wait_set_data)) {
        return true;
      }
    }
  }

  if (subscriptions) {
    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
      auto sub_data =
        static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(subscriptions->subscribers[i]);
      if (sub_data == nullptr) {
        continue;
      }
      if (sub_data->queue_has_data_and_attach_condition_if_not(wait_set_data)) {
        return true;
      }
    }
  }

  if (services) {
    for (size_t i = 0; i < services->service_count; ++i) {
      auto serv_data = static_cast<rmw_zenoh_cpp::rmw_service_data_t *>(services->services[i]);
      if (serv_data == nullptr) {
        continue;
      }
      if (serv_data->queue_has_data_and_attach_condition_if_not(wait_set_data)) {
        return true;
      }
    }
  }

  if (clients) {
    for (size_t i = 0; i < clients->client_count; ++i) {
      rmw_zenoh_cpp::rmw_client_data_t * client_data =
        static_cast<rmw_zenoh_cpp::rmw_client_data_t *>(clients->clients[i]);
      if (client_data == nullptr) {
        continue;
      }
      if (client_data->queue_has_data_and_attach_condition_if_not(wait_set_data)) {
        return true;
      }
    }
  }

  return false;
}
}  // namespace

//==============================================================================
/// Waits on sets of different entities and returns when one is ready.
rmw_ret_t
rmw_wait(
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    wait set handle,
    wait_set->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto wait_set_data = static_cast<rmw_zenoh_cpp::rmw_wait_set_data_t *>(wait_set->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    wait_set_data,
    "waitset data struct is null",
    return RMW_RET_ERROR);

  // rmw_wait should return *all* entities that have data available, and let the caller decide
  // how to handle them.
  //
  // If there is no data currently available in any of the entities we were told to wait on, we
  // we attach a context-global condition variable to each entity, calculate a timeout based on
  // wait_timeout, and then sleep on the condition variable.  If any of the entities has an event
  // during that time, it will wake up from that sleep.
  //
  // If there is data currently available in one or more of the entities, then we'll skip attaching
  // the condition variable, and skip the sleep, and instead just go to the last part.
  //
  // In the last part, we check every entity and see if there are conditions that make it ready.
  // If that entity is not ready, then we set the pointer to it to nullptr in the wait set, which
  // signals to the upper layers that it isn't ready.  If something is ready, then we leave it as
  // a valid pointer.

  bool skip_wait = check_and_attach_condition(
    subscriptions, guard_conditions, services, clients, events, wait_set_data);
  if (!skip_wait) {
    std::unique_lock<std::mutex> lock(wait_set_data->condition_mutex);

    // According to the RMW documentation, if wait_timeout is NULL that means
    // "wait forever", if it specified as 0 it means "never wait", and if it is anything else wait
    // for that amount of time.
    if (wait_timeout == nullptr) {
      wait_set_data->condition_variable.wait(
        lock, [wait_set_data]() {
          return wait_set_data->triggered;
        });
    } else {
      if (wait_timeout->sec != 0 || wait_timeout->nsec != 0) {
        wait_set_data->condition_variable.wait_for(
          lock,
          std::chrono::nanoseconds(wait_timeout->nsec + RCUTILS_S_TO_NS(wait_timeout->sec)),
          [wait_set_data]() {return wait_set_data->triggered;});
      }
    }

    // It is important to reset this here while still holding the lock, otherwise every subsequent
    // call to rmw_wait() will be immediately ready.  We could handle this another way by making
    // "triggered" a stack variable in this function and "attaching" it during
    // "check_and_attach_condition", but that isn't clearly better so leaving this.
    wait_set_data->triggered = false;
  }

  bool wait_result = false;

  // According to the documentation for rmw_wait in rmw.h, entries in the various arrays that have
  // *not* been triggered should be set to NULL
  if (guard_conditions) {
    for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
      rmw_zenoh_cpp::GuardCondition * gc =
        static_cast<rmw_zenoh_cpp::GuardCondition *>(guard_conditions->guard_conditions[i]);
      if (gc == nullptr) {
        continue;
      }
      if (!gc->detach_condition_and_is_trigger_set()) {
        // Setting to nullptr lets rcl know that this guard condition is not ready
        guard_conditions->guard_conditions[i] = nullptr;
      } else {
        wait_result = true;
      }
    }
  }

  if (events) {
    for (size_t i = 0; i < events->event_count; ++i) {
      auto event = static_cast<rmw_event_t *>(events->events[i]);
      auto event_data = static_cast<rmw_zenoh_cpp::EventsManager *>(event->data);
      if (event_data == nullptr) {
        continue;
      }

      rmw_zenoh_cpp::rmw_zenoh_event_type_t zenoh_event_type =
        rmw_zenoh_cpp::zenoh_event_from_rmw_event(event->event_type);
      if (zenoh_event_type == rmw_zenoh_cpp::ZENOH_EVENT_INVALID) {
        continue;
      }

      if (event_data->detach_condition_and_event_queue_is_empty(zenoh_event_type)) {
        // Setting to nullptr lets rcl know that this subscription is not ready
        events->events[i] = nullptr;
      } else {
        wait_result = true;
      }
    }
  }

  if (subscriptions) {
    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
      auto sub_data =
        static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(subscriptions->subscribers[i]);
      if (sub_data == nullptr) {
        continue;
      }

      if (sub_data->detach_condition_and_queue_is_empty()) {
        // Setting to nullptr lets rcl know that this subscription is not ready
        subscriptions->subscribers[i] = nullptr;
      } else {
        wait_result = true;
      }
    }
  }

  if (services) {
    for (size_t i = 0; i < services->service_count; ++i) {
      auto serv_data = static_cast<rmw_zenoh_cpp::rmw_service_data_t *>(services->services[i]);
      if (serv_data == nullptr) {
        continue;
      }

      if (serv_data->detach_condition_and_queue_is_empty()) {
        // Setting to nullptr lets rcl know that this service is not ready
        services->services[i] = nullptr;
      } else {
        wait_result = true;
      }
    }
  }

  if (clients) {
    for (size_t i = 0; i < clients->client_count; ++i) {
      rmw_zenoh_cpp::rmw_client_data_t * client_data =
        static_cast<rmw_zenoh_cpp::rmw_client_data_t *>(clients->clients[i]);
      if (client_data == nullptr) {
        continue;
      }

      if (client_data->detach_condition_and_queue_is_empty()) {
        // Setting to nullptr lets rcl know that this client is not ready
        clients->clients[i] = nullptr;
      } else {
        wait_result = true;
      }
    }
  }

  return wait_result ? RMW_RET_OK : RMW_RET_TIMEOUT;
}

//==============================================================================
/// Return the name and namespace of all nodes in the ROS graph.
rmw_ret_t
rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_names, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespaces, RMW_RET_INVALID_ARGUMENT);

  rcutils_allocator_t * allocator = &node->context->options.allocator;
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache->get_node_names(
    node_names, node_namespaces, nullptr, allocator);
}

//==============================================================================
/// Return the name, namespace, and enclave name of all nodes in the ROS graph.
rmw_ret_t
rmw_get_node_names_with_enclaves(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_names, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespaces, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(enclaves, RMW_RET_INVALID_ARGUMENT);

  rcutils_allocator_t * allocator = &node->context->options.allocator;
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache->get_node_names(
    node_names, node_namespaces, enclaves, allocator);
}

//==============================================================================
/// Count the number of known publishers matching a topic name.
rmw_ret_t
rmw_count_publishers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_TOPIC_VALID != validation_result) {
    const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache->count_publishers(topic_name, count);
}

//==============================================================================
/// Count the number of known subscribers matching a topic name.
rmw_ret_t
rmw_count_subscribers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_TOPIC_VALID != validation_result) {
    const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache->count_subscriptions(topic_name, count);
}

//==============================================================================
/// Count the number of known clients matching a service name.
rmw_ret_t
rmw_count_clients(
  const rmw_node_t * node,
  const char * service_name,
  size_t * count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  rmw_ret_t ret = rmw_validate_full_topic_name(service_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_TOPIC_VALID != validation_result) {
    const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache->count_clients(service_name, count);
}

//==============================================================================
/// Count the number of known servers matching a service name.
rmw_ret_t
rmw_count_services(
  const rmw_node_t * node,
  const char * service_name,
  size_t * count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  rmw_ret_t ret = rmw_validate_full_topic_name(service_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_TOPIC_VALID != validation_result) {
    const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache->count_services(service_name, count);
}

//==============================================================================
/// Get the globally unique identifier (GID) of a publisher.
rmw_ret_t
rmw_get_gid_for_publisher(const rmw_publisher_t * publisher, rmw_gid_t * gid)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_publisher_data_t * pub_data =
    static_cast<rmw_zenoh_cpp::rmw_publisher_data_t *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, RMW_RET_INVALID_ARGUMENT);

  gid->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  memcpy(gid->data, pub_data->pub_gid, RMW_GID_STORAGE_SIZE);

  return RMW_RET_OK;
}

//==============================================================================
/// Get the globally unique identifier (GID) of a service client.
rmw_ret_t
rmw_get_gid_for_client(const rmw_client_t * client, rmw_gid_t * gid)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_client_data_t * client_data =
    static_cast<rmw_zenoh_cpp::rmw_client_data_t *>(client->data);

  gid->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  memcpy(gid->data, client_data->client_gid, RMW_GID_STORAGE_SIZE);

  return RMW_RET_OK;
}

//==============================================================================
/// Check if two globally unique identifiers (GIDs) are equal.
rmw_ret_t
rmw_compare_gids_equal(const rmw_gid_t * gid1, const rmw_gid_t * gid2, bool * result)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(gid1, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    gid1,
    gid1->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid2, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    gid2,
    gid2->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(result, RMW_RET_INVALID_ARGUMENT);

  *result = memcmp(gid1->data, gid2->data, RMW_GID_STORAGE_SIZE) == 0;

  return RMW_RET_OK;
}

//==============================================================================
/// Check if a service server is available for the given service client.
rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * is_available)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(is_available, RMW_RET_INVALID_ARGUMENT);

  rmw_zenoh_cpp::rmw_client_data_t * client_data =
    static_cast<rmw_zenoh_cpp::rmw_client_data_t *>(client->data);
  if (client_data == nullptr) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Unable to retreive client_data from client for service %s", client->service_name);
    return RMW_RET_INVALID_ARGUMENT;
  }

  std::string service_type = client_data->request_type_support->get_name();
  size_t suffix_substring_position = service_type.find("Request_");
  if (std::string::npos != suffix_substring_position) {
    service_type = service_type.substr(0, suffix_substring_position);
  } else {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unexpected type %s for client %s. Report this bug",
      service_type.c_str(), client->service_name);
    return RMW_RET_INVALID_ARGUMENT;
  }

  return node->context->impl->graph_cache->service_server_is_available(
    client->service_name, service_type.c_str(), is_available);
}

//==============================================================================
/// Set the current log severity
rmw_ret_t
rmw_set_log_severity(rmw_log_severity_t severity)
{
  switch (severity) {
    case RMW_LOG_SEVERITY_DEBUG:
      rmw_zenoh_cpp::Logger::get().set_log_level(RCUTILS_LOG_SEVERITY_DEBUG);
      break;
    case RMW_LOG_SEVERITY_INFO:
      rmw_zenoh_cpp::Logger::get().set_log_level(RCUTILS_LOG_SEVERITY_INFO);
      break;
    case RMW_LOG_SEVERITY_WARN:
      rmw_zenoh_cpp::Logger::get().set_log_level(RCUTILS_LOG_SEVERITY_WARN);
      break;
    case RMW_LOG_SEVERITY_ERROR:
      rmw_zenoh_cpp::Logger::get().set_log_level(RCUTILS_LOG_SEVERITY_ERROR);
      break;
    case RMW_LOG_SEVERITY_FATAL:
      rmw_zenoh_cpp::Logger::get().set_log_level(RCUTILS_LOG_SEVERITY_FATAL);
      break;
    default:
      return RMW_RET_UNSUPPORTED;
  }
  return RMW_RET_OK;
}

//==============================================================================
/// Set the on new message callback function for the subscription.
rmw_ret_t
rmw_subscription_set_on_new_message_callback(
  rmw_subscription_t * subscription,
  rmw_event_callback_t callback,
  const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::rmw_subscription_data_t * sub_data =
    static_cast<rmw_zenoh_cpp::rmw_subscription_data_t *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);
  sub_data->data_callback_mgr.set_callback(
    user_data, callback);
  return RMW_RET_OK;
}

//==============================================================================
/// Set the on new request callback function for the service.
rmw_ret_t
rmw_service_set_on_new_request_callback(
  rmw_service_t * service,
  rmw_event_callback_t callback,
  const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::rmw_service_data_t * service_data =
    static_cast<rmw_zenoh_cpp::rmw_service_data_t *>(service->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);
  service_data->data_callback_mgr.set_callback(
    user_data, callback);
  return RMW_RET_OK;
}

//==============================================================================
/// Set the on new response callback function for the client.
rmw_ret_t
rmw_client_set_on_new_response_callback(
  rmw_client_t * client,
  rmw_event_callback_t callback,
  const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::rmw_client_data_t * client_data =
    static_cast<rmw_zenoh_cpp::rmw_client_data_t *>(client->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);
  client_data->data_callback_mgr.set_callback(
    user_data, callback);
  return RMW_RET_OK;
}
}  // extern "C"
