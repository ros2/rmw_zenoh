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

#ifdef __cplusplus
}
#endif

#endif  // RMW_ZENOH_COMMON_CPP__RMW_ZENOH_COMMON_H_
