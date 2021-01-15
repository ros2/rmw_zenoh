// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_ZENOH_COMMON_CPP__RMW_ZENOH_COMMON_H_
#define RMW_ZENOH_COMMON_CPP__RMW_ZENOH_COMMON_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "rmw/event.h"

rmw_ret_t
rmw_zenoh_common_init_pre(
  const rmw_init_options_t * options, rmw_context_t * context,
  const char * const eclipse_zenoh_identifier);

rmw_node_t *
rmw_zenoh_common_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_,
  size_t domain_id,
  bool localhost_only,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_shutdown(rmw_context_t * context, const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_init_options_init(
  rmw_init_options_t * init_options, rcutils_allocator_t allocator,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_context_fini(rmw_context_t * context, const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_destroy_node(rmw_node_t * node, const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_init_options_copy(
  const rmw_init_options_t * src, rmw_init_options_t * dst,
  const char * const eclipse_zenoh_identifier);

rmw_guard_condition_t *
rmw_zenoh_common_create_guard_condition(
  rmw_context_t * context,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_init_options_fini(
  rmw_init_options_t * init_options,
  const char * const eclipse_zenoh_identifier);

rmw_subscription_t *
rmw_zenoh_common_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_subscription_options_t * subscription_options,
  const char * const eclipse_zenoh_identifier);

/// DESTROY SUBSCRIPTION =======================================================
// Destroy and deallocate an RMW subscription
rmw_ret_t
rmw_zenoh_common_destroy_subscription(
  rmw_node_t * node, rmw_subscription_t * subscription,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos_profile,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * count,
  const char * const eclipse_zenoh_identifier);

/// CREATE PUBLISHER ===========================================================
// Create and return an rmw publisher.
rmw_publisher_t *
rmw_zenoh_common_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options,
  const char * const eclipse_zenoh_identifier);

/// DESTROY PUBLISHER ==========================================================
// Destroy and deallocate an rmw publisher.
rmw_ret_t
rmw_zenoh_common_destroy_publisher(
  rmw_node_t * node, rmw_publisher_t * publisher,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos_profile,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_publisher_event_init(
  rmw_event_t * event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_subscription_event_init(
  rmw_event_t * event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation);

rmw_client_t *
rmw_zenoh_common_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile,
  const char * const eclipse_zenoh_identifier);

rmw_service_t *
rmw_zenoh_common_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * result,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_get_gid_for_publisher(
  const rmw_publisher_t * publisher,
  rmw_gid_t * gid,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_destroy_wait_set(
  rmw_wait_set_t * wait_set,
  const char * const eclipse_zenoh_identifier);

rmw_wait_set_t *
rmw_zenoh_common_create_wait_set(
  rmw_context_t * context,
  size_t max_conditions,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_trigger_guard_condition(
  const rmw_guard_condition_t * guard_condition_handle,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header,
  void * ros_response,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_take_request(
  const rmw_service_t * service,
  rmw_service_info_t * request_header,
  void * ros_request,
  bool * taken,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_destroy_service(
  rmw_node_t * node, rmw_service_t * service,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_take_response(
  const rmw_client_t * client,
  rmw_service_info_t * request_header,
  void * ros_response,
  bool * taken,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_send_request(
  const rmw_client_t * client,
  const void * ros_request,
  int64_t * sequence_id,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_destroy_client(
  rmw_node_t * node,
  rmw_client_t * client,
  const char * const eclipse_zenoh_identifier);

rmw_ret_t
rmw_zenoh_common_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message);

#ifdef __cplusplus
}
#endif

#endif  // RMW_ZENOH_COMMON_CPP__RMW_ZENOH_COMMON_H_
