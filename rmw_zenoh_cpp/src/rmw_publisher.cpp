#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"

#include <rmw/validate_full_topic_name.h>
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/rmw.h"

#include "rmw_zenoh_cpp/rmw_context_impl.hpp"
#include "rmw_zenoh_cpp/identifier.hpp"

#include "impl/type_support_common.hpp"
#include "impl/pubsub_impl.hpp"

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
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "[rmw_create_publisher] %s", topic_name);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr
  );

  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  if (strlen(topic_name) == 0) {
    RMW_SET_ERROR_MSG("publisher topic is empty string");
    return nullptr;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);

  // TODO(CH3): When we figure out how to spoof QoS, check for a 'configured' QoS to pass the final
  // test that is failing
  //
  // if (# SOME CHECK HERE) {
  //   RMW_SET_ERROR_MSG("expected configured QoS profile");
  //   return nullptr;
  // }

  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // VALIDATE TOPIC NAME =======================================================
  int * validation_result = static_cast<int *>(
    allocator->allocate(sizeof(int), allocator->state)
  );

  if (rmw_validate_full_topic_name(topic_name, validation_result, nullptr) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("rmw_validate_full_topic_name failed!");
    return nullptr;
  }

  if (*validation_result == RMW_TOPIC_VALID || qos_profile->avoid_ros_namespace_conventions) {
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

  publisher->options = *publisher_options;

  // CREATE PUBLISHER MEMBERS ==================================================
  // Get typed pointer to implementation specific publisher data struct
  auto publisher_data = static_cast<rmw_publisher_data_t *>(publisher->data);

  // Init type support callbacks
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);

  // Create Zenoh resource
  ZNSession * s = node->context->impl->session;

  // NOTE(CH3): This topic ID only unique WITHIN this process!
  //
  // Another topic on another process might clash with the ID on this process, even within the
  // same Zenoh network! It is not a UUID!!
  publisher_data->zn_topic_id_ = zn_declare_resource(s, publisher->topic_name);

  // Assign publisher data members
  publisher_data->zn_session_ = s;
  publisher_data->typesupport_identifier_ = type_support->typesupport_identifier;
  publisher_data->type_support_impl_ = type_support->data;
  // RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "[rmw_create_publisher] Zenoh resource declared: %s (%ld)", topic_name, publisher_data->zn_topic_id);

  // Allocate and in-place assign new message typesupport instance
  publisher_data->type_support_ = static_cast<MessageTypeSupport_cpp *>(
    allocator->allocate(sizeof(MessageTypeSupport_cpp), allocator->state)
  );
  new(publisher_data->type_support_) MessageTypeSupport_cpp(callbacks);
  if (!publisher_data->type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate MessageTypeSupport");
    allocator->deallocate(publisher_data->type_support_, allocator->state);
    allocator->deallocate(publisher->data, allocator->state);
    allocator->deallocate(publisher, allocator->state);
    return nullptr;
  }

  // Assign node pointer
  publisher_data->node_ = node;

  // TODO(CH3): Put the publisher name/pointer into its corresponding node for tracking?

  // NOTE(CH3) TODO(CH3): No graph updates are implemented yet
  // I am not sure how this will work with Zenoh
  //
  // Perhaps track something using the nodes?

  return publisher;
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

  // OBTAIN ALLOCATOR ==========================================================
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
