#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"

#include <rmw/validate_full_topic_name.h>
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/rmw.h"

#include "rmw_zenoh_cpp/rmw_context_impl.hpp"
#include "rmw_zenoh_cpp/identifier.hpp"

#include "type_support_common.hpp"
#include "pubsub_impl.hpp"

extern "C"
{
#include "zenoh/zenoh-ffi.h"

/// CREATE PUBLISHER ===========================================================
// Create and return an rmw publisher.
rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options)
{
  // ASSERTIONS ================================================================
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node,
    "node handle is null",
    return nullptr
  );

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr
  );

  if (!topic_name || strlen(topic_name) == 0) {
    RMW_SET_ERROR_MSG("publisher topic is null or empty string");
    return nullptr;
  }

  if (!qos_profile) {  // Check to pass tests
    RMW_SET_ERROR_MSG("qos_profile is null");
    return nullptr;
  }

  // NOTE(CH3): For some reason the tests want a failed publisher init on passing an unknown QoS.
  // I don't understand why, and I don't see a check to fulfill that test in any RMW implementations
  // if (# SOME CHECK HERE) {
  //   RMW_SET_ERROR_MSG("expected configured QoS profile");
  //   return nullptr;
  // }

  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher_options,
    "publisher_options is null",
    return nullptr
  );

  RMW_CHECK_FOR_NULL_WITH_MSG(
    type_supports,
    "type_supports is null",
    return nullptr
  );

  // ASSIGN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // VALIDATE TOPIC NAME =======================================================
  int * validation_result = static_cast<int *>(
    allocator->allocate(sizeof(int), allocator->state)
  );

  rmw_validate_full_topic_name(topic_name, validation_result, nullptr);

  if (*validation_result == RMW_TOPIC_VALID
      || qos_profile->avoid_ros_namespace_conventions) {
    allocator->deallocate(validation_result, allocator->state);
  } else {
    RMW_SET_ERROR_MSG("publisher topic is malformed!");
    allocator->deallocate(validation_result, allocator->state);
    return nullptr;
  }

  // OBTAIN TYPESUPPORT ========================================================
  const rosidl_message_type_support_t * type_support = get_message_typesupport_handle(
    type_supports, RMW_ZENOH_CPP_TYPESUPPORT_C
  );

  if (!type_support) {
    type_support = get_message_typesupport_handle(
      type_supports, RMW_ZENOH_CPP_TYPESUPPORT_CPP);
    if (!type_support) {
      RCUTILS_LOG_INFO("%s", topic_name);
      RMW_SET_ERROR_MSG("type support not from this implementation");
      return nullptr;
    }
  }

  // CREATE PUBLISHER ==========================================================
  rmw_publisher_t * publisher = static_cast<rmw_publisher_t *>(
    allocator->allocate(sizeof(rmw_publisher_t), allocator->state)
  );
  if (!publisher) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_publisher_t");
    allocator->deallocate(publisher, allocator->state);
    return nullptr;
  }

  // Populate common members
  publisher->implementation_identifier = eclipse_zenoh_identifier;  // const char * assignment
  if (!publisher->implementation_identifier) {
    RMW_SET_ERROR_MSG("failed to allocate implementation identifier");
    allocator->deallocate(publisher, allocator->state);
    return nullptr;
  }

  publisher->topic_name = rcutils_strdup(topic_name, *allocator);
  if (!publisher->topic_name) {
    RMW_SET_ERROR_MSG("failed to allocate publisher topic name");
    allocator->deallocate(publisher, allocator->state);
    return nullptr;
  }

  publisher->data = static_cast<rmw_publisher_data_t *>(
    allocator->allocate(sizeof(rmw_publisher_data_t), allocator->state)
  );
  if (!publisher->data) {
    RMW_SET_ERROR_MSG("failed to allocate publisher data");
    allocator->deallocate(publisher->data, allocator->state);
    allocator->deallocate(publisher, allocator->state);
    return nullptr;
  }

  // CREATE PUBLISHER MEMBERS ==================================================
  // Init type support callbacks
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);

  // Create Zenoh resource
  ZNSession * s = node->context->impl->session;
  size_t zn_topic_id = zn_declare_resource(s, topic_name);

  // Create implementation specific publisher struct
  rmw_publisher_data_t * publisher_data = static_cast<rmw_publisher_data_t *>(
    allocator->allocate(sizeof(rmw_publisher_data_t), allocator->state)
  );
  if (!publisher_data) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_publisher_data_t");
    goto cleanup_data;
    return nullptr;
  }

  publisher_data->zn_session_ = static_cast<ZNSession *>(
    allocator->allocate(sizeof(ZNSession *), allocator->state)
  );
  publisher_data->zn_session_ = s;
  if (!publisher_data->zn_session_) {
    RMW_SET_ERROR_MSG("failed to allocate Zenoh session");
    goto cleanup_typesupport;
    return nullptr;
  }

  RCUTILS_LOG_INFO_NAMED("rmw_publish", "topic: %s, id: %ld", topic_name, zn_topic_id);

  publisher_data->zn_topic_id_ = zn_topic_id;
  if (!publisher_data->zn_topic_id_) {
    RMW_SET_ERROR_MSG("failed to allocate Zenoh topic ID");
    goto cleanup_typesupport;
    return nullptr;
  }

  publisher_data->typesupport_identifier_ = type_support->typesupport_identifier;
  if (!publisher_data->typesupport_identifier_) {
    RMW_SET_ERROR_MSG("failed to allocate typesupport_identifier_");
    goto cleanup_typesupport;
    return nullptr;
  }

  publisher_data->type_support_impl_ = type_support->data;
  if (!publisher_data->type_support_impl_) {
    RMW_SET_ERROR_MSG("failed to allocate type_support_impl_");
    goto cleanup_typesupport;
    return nullptr;
  }

  publisher_data->type_support_ = static_cast<MessageTypeSupport_cpp *>(
    allocator->allocate(sizeof(MessageTypeSupport_cpp *), allocator->state)
  );
  publisher_data->type_support_ = new (std::nothrow) MessageTypeSupport_cpp(callbacks);
  if (!publisher_data->type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate MessageTypeSupport");
    goto cleanup_typesupport;
    return nullptr;
  }

  publisher->data = publisher_data;
  if (!publisher->data) {
    RMW_SET_ERROR_MSG("failed to allocate publisher data");
    goto cleanup_typesupport;
    return nullptr;
  }

  // NOTE(CH3) TODO(CH3): No graph updates are implemented yet
  // I am not sure how this will work with Zenoh
  //
  // Perhaps track something using the nodes?

  return publisher;

cleanup_typesupport:
  allocator->deallocate(publisher_data->type_support_, allocator->state);

cleanup_data:
  allocator->deallocate(publisher_data, allocator->state);

  allocator->deallocate(publisher->data, allocator->state);
  allocator->deallocate(publisher, allocator->state);

  return nullptr;
}

/// DESTROY PUBLISHER ==========================================================
// Destroy and deallocate an rmw publisher.
rmw_ret_t
rmw_destroy_publisher(rmw_node_t * node, rmw_publisher_t * publisher)
{
  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );

  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );

  // ASSIGN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // CLEANUP ===================================================================
  allocator->deallocate(
    static_cast<rmw_publisher_data_t *>(publisher->data)->type_support_, allocator->state
  );

  allocator->deallocate(publisher->data, allocator->state);
  allocator->deallocate(publisher, allocator->state);

  return RMW_RET_OK;
}

/// UNIMPLEMENTED ==============================================================
rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_publisher_allocation_t * allocation)
{
  (void)type_support;
  (void)message_bounds;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_init_publisher_allocation");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_fini_publisher_allocation(rmw_publisher_allocation_t * allocation)
{
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_fini_publisher_allocation");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message)
{
  (void)publisher;
  (void)type_support;
  (void)ros_message;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_borrow_loaned_message");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_return_loaned_message_from_publisher(const rmw_publisher_t * publisher, void * loaned_message)
{
  (void)publisher;
  (void)loaned_message;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_return_loaned_message_from_publisher");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_publisher_count_matched_subscriptions(const rmw_publisher_t * publisher, size_t * count)
{
  (void)publisher;
  (void)count;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publisher_count_matched_subscriptions");
  return RMW_RET_ERROR;
}

// STUB
rmw_ret_t
rmw_publisher_get_actual_qos(const rmw_publisher_t * publisher, rmw_qos_profile_t * qos_profile)
{
  (void)publisher;
  (void)qos_profile;
  // RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publisher_get_actual_qos");
  // return RMW_RET_ERROR;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_assert_liveliness(const rmw_publisher_t * publisher)
{
  (void)publisher;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publisher_assert_liveliness");
  return RMW_RET_ERROR;
}

} // extern "C"
