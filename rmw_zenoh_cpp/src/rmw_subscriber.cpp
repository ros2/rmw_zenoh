#include <functional>

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

/// CREATE SUBSCRIPTION ========================================================
// Create and return an rmw subscriber.
rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_subscription_options_t * subscription_options)
{
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_create_subscription");
  RCUTILS_LOG_INFO("NODE_NAME: %s", node->name);

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
    RMW_SET_ERROR_MSG("subscription topic is empty string");
    return nullptr;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);

  // NOTE(CH3): For some reason the tests want a failed publisher init on passing an unknown QoS.
  // I don't understand why, and I don't see a check to fulfill that test in any RMW implementations
  // if (# SOME CHECK HERE) {
  //   RMW_SET_ERROR_MSG("expected configured QoS profile");
  //   return nullptr;
  // }

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_options, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);

  // OBTAIN ALLOCATOR ==========================================================
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
    RMW_SET_ERROR_MSG("subscription topic is malformed!");
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

  // CREATE SUBSCRIPTION ==========================================================
  rmw_subscription_t * subscription = static_cast<rmw_subscription_t *>(
    allocator->allocate(sizeof(rmw_subscription_t), allocator->state)
  );
  if (!subscription) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_subscription_t");
    allocator->deallocate(subscription, allocator->state);
    return nullptr;
  }

  // Populate common members
  subscription->implementation_identifier = eclipse_zenoh_identifier;  // const char * assignment
  subscription->options = *subscription_options;
  subscription->can_loan_messages = false;

  subscription->topic_name = rcutils_strdup(topic_name, *allocator);
  if (!subscription->topic_name) {
    RMW_SET_ERROR_MSG("failed to allocate subscription topic name");
    allocator->deallocate(subscription, allocator->state);
    return nullptr;
  }

  subscription->data = static_cast<rmw_subscription_data_t *>(
    allocator->allocate(sizeof(rmw_subscription_data_t), allocator->state)
  );
  if (!subscription->data) {
    RMW_SET_ERROR_MSG("failed to allocate subscription data");
    allocator->deallocate(subscription->data, allocator->state);
    allocator->deallocate(subscription, allocator->state);
    return nullptr;
  }

  // CREATE SUBSCRIPTION MEMBERS ===============================================
  // Init type support callbacks
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);

  // Create Zenoh resource
  ZNSession * s = node->context->impl->session;

  // Create implementation specific subscription struct
  rmw_subscription_data_t * subscription_data = static_cast<rmw_subscription_data_t *>(
    allocator->allocate(sizeof(rmw_subscription_data_t), allocator->state)
  );
  if (!subscription_data) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_subscription_data_t");
    goto cleanup_data;
    return nullptr;
  }

  subscription_data->zn_session_ = static_cast<ZNSession *>(
    allocator->allocate(sizeof(ZNSession *), allocator->state)
  );
  subscription_data->zn_session_ = s;
  if (!subscription_data->zn_session_) {
    RMW_SET_ERROR_MSG("failed to allocate Zenoh session");
    goto cleanup_session;
    return nullptr;
  }

  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_create_subscription topic: %s", topic_name);

  subscription_data->typesupport_identifier_ = type_support->typesupport_identifier;
  if (!subscription_data->typesupport_identifier_) {
    RMW_SET_ERROR_MSG("failed to allocate typesupport_identifier_");
    goto cleanup_session;
    return nullptr;
  }

  subscription_data->type_support_impl_ = type_support->data;
  if (!subscription_data->type_support_impl_) {
    RMW_SET_ERROR_MSG("failed to allocate type_support_impl_");
    goto cleanup_session;
    return nullptr;
  }

  subscription_data->type_support_ = static_cast<MessageTypeSupport_cpp *>(
    allocator->allocate(sizeof(MessageTypeSupport_cpp *), allocator->state)
  );
  subscription_data->type_support_ = new (std::nothrow) MessageTypeSupport_cpp(callbacks);
  if (!subscription_data->type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate MessageTypeSupport");
    goto cleanup_typesupport;
    return nullptr;
  }

  // NOTE(CH3): Memory already allocated for node.
  // Might have to copy, but unsure
  subscription_data->node_ = node;
  if (!subscription_data->node_) {
    RMW_SET_ERROR_MSG("failed to assign node pointer");
    goto cleanup_typesupport;
    return nullptr;
  }

  subscription->data = subscription_data;
  if (!subscription->data) {
    RMW_SET_ERROR_MSG("failed to allocate subscription data");
    goto cleanup_typesupport;
    return nullptr;
  }

  zn_declare_subscriber(subscription_data->zn_session_,
                        subscription->topic_name,
                        zn_subinfo_default(),  // NOTE(CH3): Default for now
                        subscription_data->zn_sub_callback);

  // TODO(CH3): Put the subscription name/pointer into its corresponding node for tracking?

  // NOTE(CH3) TODO(CH3): No graph updates are implemented yet
  // I am not sure how this will work with Zenoh
  //
  // Perhaps track something using the nodes?

  return subscription;

cleanup_typesupport:
  allocator->deallocate(subscription_data->type_support_, allocator->state);

cleanup_session:
  allocator->deallocate(subscription_data->zn_session_, allocator->state);

cleanup_data:
  allocator->deallocate(subscription_data, allocator->state);

  allocator->deallocate(subscription->data, allocator->state);
  allocator->deallocate(subscription, allocator->state);

  return nullptr;
}

/// DESTROY SUBSCRIPTION
// Destroy and deallocate an RMW subscription
rmw_ret_t
rmw_destroy_subscription(rmw_node_t * node, rmw_subscription_t * subscription)
{
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_destroy_subscription");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // CLEANUP ===================================================================
  allocator->deallocate(
    static_cast<rmw_subscription_data_t *>(subscription->data)->type_support_, allocator->state
  );
  allocator->deallocate(
    static_cast<rmw_subscription_data_t *>(subscription->data)->zn_session_, allocator->state
  );
  allocator->deallocate(subscription->data, allocator->state);
  allocator->deallocate(subscription, allocator->state);

  return RMW_RET_OK;
}

rmw_ret_t
rmw_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)allocation;
  *taken = false;

  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );

  // OBTAIN SUBSCRIPTION MEMBERS ===============================================
  const char * topic_name = subscription->topic_name;
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);

  rmw_subscription_data_t * subscription_data = static_cast<rmw_subscription_data_t *>(
    subscription->data
  );
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_data, RMW_RET_ERROR);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator =
    &static_cast<rmw_subscription_data_t *>(subscription->data)->node_->context->options.allocator;

  // RETRIEVE SERIALIZED MESSAGE ===============================================
  if (subscription_data->zn_messages_.find(topic_name) == subscription_data->zn_messages_.end()) {
    RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "Message not found from: %s", topic_name);
    return RMW_RET_OK;
  }

  // DESERIALIZE MESSAGE =======================================================
  auto msg_bytes = subscription_data->zn_messages_[std::string(topic_name)];
  size_t data_length = msg_bytes.second;

  unsigned char * cdr_buffer = static_cast<unsigned char *>(
    allocator->allocate(msg_bytes.second, allocator->state)
  );
  memcpy(cdr_buffer, &msg_bytes.first.front(), msg_bytes.second);

  // Remove stored message after successful retrieval
  subscription_data->zn_messages_.erase(topic_name);

  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(cdr_buffer),
    data_length
  );  // Object that manages the raw buffer.

  eprosima::fastcdr::Cdr deser(fastbuffer,
                               eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                               eprosima::fastcdr::Cdr::DDS_CDR);  // Object that serializes the data.
  if (!subscription_data->type_support_->deserializeROSmessage(deser,
                                                               ros_message,
                                                               subscription_data->type_support_impl_)) {
    RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "COULD NOT DESERIALIZE MESSAGE");
    return RMW_RET_ERROR;
  }

  *taken = true;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)allocation;
  *taken = false;

  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take_with_info");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );

  // OBTAIN SUBSCRIPTION MEMBERS ===============================================
  const char * topic_name = subscription->topic_name;
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_ERROR);

  rmw_subscription_data_t * subscription_data = static_cast<rmw_subscription_data_t *>(
    subscription->data
  );
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_data, RMW_RET_ERROR);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator =
    &static_cast<rmw_subscription_data_t *>(subscription->data)->node_->context->options.allocator;

  // RETRIEVE SERIALIZED MESSAGE ===============================================
  if (subscription_data->zn_messages_.find(topic_name) == subscription_data->zn_messages_.end()) {
    RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "Message not found from: %s", topic_name);
    return RMW_RET_OK;
  }

  // DESERIALIZE MESSAGE =======================================================
  auto msg_bytes = subscription_data->zn_messages_[std::string(topic_name)];
  size_t data_length = msg_bytes.second;

  unsigned char * cdr_buffer = static_cast<unsigned char *>(
    allocator->allocate(msg_bytes.second, allocator->state)
  );
  memcpy(cdr_buffer, &msg_bytes.first.front(), msg_bytes.second);

  // Remove stored message after successful retrieval
  subscription_data->zn_messages_.erase(topic_name);

  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(cdr_buffer),
    data_length
  );  // Object that manages the raw buffer.

  eprosima::fastcdr::Cdr deser(fastbuffer,
                               eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                               eprosima::fastcdr::Cdr::DDS_CDR);  // Object that serializes the data.
  if (!subscription_data->type_support_->deserializeROSmessage(deser,
                                                               ros_message,
                                                               subscription_data->type_support_impl_)) {
    RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "COULD NOT DESERIALIZE MESSAGE");
    return RMW_RET_ERROR;
  }

  *taken = true;
  return RMW_RET_OK;
}

/// UNIMPLEMENTED ==============================================================
rmw_ret_t
rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_subscription_allocation_t * allocation)
{
  (void)type_support;
  (void)message_bounds;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_init_subscription_allocation");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_fini_subscription_allocation(rmw_subscription_allocation_t * allocation)
{
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_fini_subscription_allocation");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_subscription_count_matched_publishers(const rmw_subscription_t * subscription, size_t * count)
{
  (void)subscription;
  (void)count;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_subscription_count_matched_publishers");
  return RMW_RET_ERROR;
}

// STUB
rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos_profile)
{
  (void)subscription;
  (void)qos_profile;
  // RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_subscription_get_actual_qos");
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)serialized_message;
  (void)taken;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take_serialized_message");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)serialized_message;
  (void)taken;
  (void)message_info;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take_serialized_message");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_take_loaned_message(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)loaned_message;
  (void)taken;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take_loaned_message (Unsupported)");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)loaned_message;
  (void)taken;
  (void)message_info;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take_loaned_message_with_info (Unsupported)");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message)
{
  (void)subscription;
  (void)loaned_message;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_return_loaned_message_from_subscription");
  return RMW_RET_ERROR;
}

} // extern "C"
