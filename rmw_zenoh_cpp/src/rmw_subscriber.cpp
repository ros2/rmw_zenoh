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

#include "impl/type_support_common.hpp"
#include "impl/pubsub_impl.hpp"

extern "C"
{
#include "zenoh/zenoh-ffi.h"

/// CREATE SUBSCRIPTION ========================================================
// Create and return an rmw subscriber
rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_subscription_options_t * subscription_options)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "[rmw_create_subscription] %s", topic_name);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(node,
                                   node->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return nullptr);

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
  int validation_result;

  if (rmw_validate_full_topic_name(topic_name, &validation_result, nullptr) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("rmw_validate_full_topic_name failed");
    return nullptr;
  }

  if (validation_result != RMW_TOPIC_VALID && !qos_profile->avoid_ros_namespace_conventions) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("subscription topic is malformed: %s", topic_name);
    return nullptr;
  }

  // OBTAIN TYPESUPPORT ========================================================
  const rosidl_message_type_support_t * type_support = get_message_typesupport_handle(
      type_supports, RMW_ZENOH_CPP_TYPESUPPORT_C);

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
      allocator->allocate(sizeof(rmw_subscription_t), allocator->state));
  if (!subscription) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_subscription_t");
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
      allocator->allocate(sizeof(rmw_subscription_data_t), allocator->state));
  if (!subscription->data) {
    RMW_SET_ERROR_MSG("failed to allocate subscription data");
    allocator->deallocate(const_cast<char *>(subscription->topic_name), allocator->state);
    allocator->deallocate(subscription, allocator->state);
    return nullptr;
  }

  // CREATE SUBSCRIPTION MEMBERS ===============================================
  // Init type support callbacks
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);

  // Obtain Zenoh session
  ZNSession * s = node->context->impl->session;

  // Get typed pointer to implementation specific subscription data struct
  auto * subscription_data = static_cast<rmw_subscription_data_t *>(subscription->data);

  subscription_data->zn_session_ = s;
  subscription_data->typesupport_identifier_ = type_support->typesupport_identifier;
  subscription_data->type_support_impl_ = type_support->data;

  // Allocate and in-place assign new message typesupport instance
  subscription_data->type_support_ = static_cast<rmw_zenoh_cpp::MessageTypeSupport *>(
      allocator->allocate(sizeof(rmw_zenoh_cpp::MessageTypeSupport), allocator->state));
  new(subscription_data->type_support_) rmw_zenoh_cpp::MessageTypeSupport(callbacks);
  if (!subscription_data->type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate MessageTypeSupport");
    allocator->deallocate(subscription->data, allocator->state);

    allocator->deallocate(const_cast<char *>(subscription->topic_name), allocator->state);
    allocator->deallocate(subscription, allocator->state);
    return nullptr;
  }

  // Assign node pointer
  subscription_data->node_ = node;

  // Init Zenoh subscriber
  subscription_data->zn_subscriber_ = zn_declare_subscriber(
      subscription_data->zn_session_,
      subscription->topic_name,
      zn_subinfo_default(),  // NOTE(CH3): Default for now
      subscription_data->zn_sub_callback);

  // TODO(CH3): Put the subscription name/pointer into its corresponding node for tracking?

  // NOTE(CH3) TODO(CH3): No graph updates are implemented yet
  // I am not sure how this will work with Zenoh
  //
  // Perhaps track something using the nodes?

  return subscription;
}

/// DESTROY SUBSCRIPTION =======================================================
// Destroy and deallocate an RMW subscription
rmw_ret_t
rmw_destroy_subscription(rmw_node_t * node, rmw_subscription_t * subscription)
{
  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(node,
                                   node->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(subscription,
                                   subscription->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "[rmw_destroy_subscription] %s",
                          subscription->topic_name);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // CLEANUP ===================================================================
  zn_undeclare_subscriber(
      static_cast<rmw_subscription_data_t *>(subscription->data)->zn_subscriber_);

  allocator->deallocate(static_cast<rmw_subscription_data_t *>(subscription->data)->type_support_,
                        allocator->state);
  allocator->deallocate(subscription->data, allocator->state);

  allocator->deallocate(const_cast<char *>(subscription->topic_name), allocator->state);
  allocator->deallocate(subscription, allocator->state);

  return RMW_RET_OK;
}

/// TAKE MESSAGE ===============================================================
// Take message out of the message queue
rmw_ret_t
rmw_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)allocation;
  *taken = false;

  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_take");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(subscription,
                                   subscription->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_INVALID_ARGUMENT);

  // OBTAIN SUBSCRIPTION MEMBERS ===============================================
  auto subscription_data = static_cast<rmw_subscription_data_t *>(subscription->data);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &subscription_data->node_->context->options.allocator;

  // RETRIEVE SERIALIZED MESSAGE ===============================================
  std::string key(subscription->topic_name);

  if (subscription_data->zn_messages_.find(key) == subscription_data->zn_messages_.end()) {
    // NOTE(CH3): It is correct to be returning RMW_RET_OK. The information that the message
    // was not found is encoded in the fact that the taken-out parameter is still False.
    //
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "[rmw_take] Message found: %s", key.c_str());

  // DESERIALIZE MESSAGE =======================================================
  auto msg_bytes = subscription_data->zn_messages_[key];

  unsigned char * cdr_buffer = static_cast<unsigned char *>(
      allocator->allocate(msg_bytes.size(), allocator->state));
  memcpy(cdr_buffer, &msg_bytes.front(), msg_bytes.size());

  // Remove stored message after successful retrieval
  subscription_data->zn_messages_.erase(key);

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char *>(cdr_buffer), msg_bytes.size());

  // Object that serializes the data
  eprosima::fastcdr::Cdr deser(fastbuffer,
                               eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                               eprosima::fastcdr::Cdr::DDS_CDR);
  if (!subscription_data->type_support_->deserializeROSmessage(
      deser, ros_message, subscription_data->type_support_impl_)) {
    RMW_SET_ERROR_MSG("could not deserialize ROS message");
    return RMW_RET_ERROR;
  }

  *taken = true;
  return RMW_RET_OK;
}

/// TAKE MESSAGE WITH INFO =====================================================
// Take message out of the message queue, and obtain its message info
//
// TODO(CH3): By right the passed rmw_message_info_t should have its members filled, which include:
// Source and message reception timestamps, publisher_gid, and whether the message was from
// inside the process.
//
// The problem is I'm not sure how to get this data from the way we've done things...
// So this functionality is left unimplemented for now. It doesn't seem to break pubsub.
//
// (More specifically, there isn't a way to send the information on the publish side using Zenoh
// unless we include it in the raw message bytes that get sent.)
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

  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_take_with_info");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(subscription,
                                   subscription->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);

  // OBTAIN SUBSCRIPTION MEMBERS ===============================================
  auto subscription_data = static_cast<rmw_subscription_data_t *>(subscription->data);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &subscription_data->node_->context->options.allocator;

  // RETRIEVE SERIALIZED MESSAGE ===============================================
  std::string key(subscription->topic_name);

  if (subscription_data->zn_messages_.find(key)
      == subscription_data->zn_messages_.end()) {
    return RMW_RET_OK;
  }
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "[rmw_take_with_info] Message found: %s", key.c_str());

  // DESERIALIZE MESSAGE =======================================================
  auto msg_bytes = subscription_data->zn_messages_[key];

  unsigned char * cdr_buffer = static_cast<unsigned char *>(
      allocator->allocate(msg_bytes.size(), allocator->state));
  memcpy(cdr_buffer, &msg_bytes.front(), msg_bytes.size());

  // Remove stored message after successful retrieval
  subscription_data->zn_messages_.erase(key);

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char *>(cdr_buffer), msg_bytes.size());

  // Object that serializes the data
  eprosima::fastcdr::Cdr deser(fastbuffer,
                               eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                               eprosima::fastcdr::Cdr::DDS_CDR);
  if (!subscription_data->type_support_->deserializeROSmessage(
      deser, ros_message, subscription_data->type_support_impl_)) {
    RMW_SET_ERROR_MSG("could not deserialize message");
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
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_subscription_get_actual_qos (STUB)");
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
