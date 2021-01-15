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

// Doc: http://docs.ros2.org/latest/api/rmw/init_8h.html

#include <cstring>

#include <memory>

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/init.h"

#include "rcutils/logging_macros.h"

#include "rmw_zenoh_common_cpp/rmw_context_impl.hpp"
#include "rmw_zenoh_common_cpp/rmw_init_options_impl.hpp"

#include "rmw_zenoh_common_cpp/identifier.hpp"

#include "rmw_zenoh_common_cpp/rmw_zenoh_common.h"
#include "rmw_zenoh_common_cpp/zenoh-net-interface.h"

zn_properties_t * configure_connection_mode(rmw_context_t * context)
{
  if (strcmp(context->options.impl->mode, "CLIENT") == 0) {
    return zn_config_client(context->options.impl->session_locator);
  } else {
    return zn_config_peer();
  }
}

void configure_session(zn_session_t * session)
{
  (void)session;
}

const char *
rmw_get_implementation_identifier()
{
  return eclipse_zenoh_identifier;
}

const char *
rmw_get_serialization_format()
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "rmw_get_serialization_format");
  return nullptr;
}

/// INIT CONTEXT ===============================================================
// Initialize the middleware with the given options, and yielding an context.
//
// rmw_context_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__context__t.html
//
// Starts a new Zenoh session and configures it according to the init options
// with the following environment variables:
//  - RMW_ZENOH_SESSION_LOCATOR: Session TCP locator to use
//  - RMW_ZENOH_MODE: Lets you set the session to be in CLIENT, ROUTER, or PEER mode
//                    (defaults to PEER)
rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_init");

  rmw_ret_t ret = rmw_zenoh_common_init_pre(options, context, eclipse_zenoh_identifier);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  // Create implementation specific context
  rcutils_allocator_t * allocator = &context->options.allocator;

  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(
    allocator->allocate(sizeof(rmw_context_impl_t), allocator->state));

  if (!context_impl) {
    RMW_SET_ERROR_MSG("failed to allocate context impl");
    *context = rmw_get_zero_initialized_context();
    return RMW_RET_BAD_ALLOC;
  }

  // Open configured Zenoh session, then assign it to the context
  zn_properties_t * config = configure_connection_mode(context);

  if (nullptr == config) {
    allocator->deallocate(context_impl, allocator->state);
    *context = rmw_get_zero_initialized_context();
    return RMW_RET_ERROR;
  }

  zn_session_t * session = zn_open(config);

  if (session == nullptr) {
    RMW_SET_ERROR_MSG("failed to create Zenoh session when starting context");
    allocator->deallocate(context_impl, allocator->state);
    *context = rmw_get_zero_initialized_context();
    return RMW_RET_ERROR;
  } else {
    context_impl->session = session;
    context_impl->is_shutdown = false;
  }

  // CLEANUP IF PASSED =========================================================
  context->impl = context_impl;

  configure_session(context_impl->session);
  return RMW_RET_OK;
}

/// CREATE NODE ================================================================
// Create a node and return a handle to that node.
//
// rmw_node_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__node__t.html
//
// In the case of Zenoh, the only relevant members are name, namespace and implementation
// identifier.
//
// Most likely we will associate a subset of the context session's publishers and subscribers to
// individual nodes, even though to Zenoh it looks like the session is the one holding on to
// all of them.
rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_,
  size_t domain_id,
  bool localhost_only)
{
  return rmw_zenoh_common_create_node(
    context,
    name,
    namespace_,
    domain_id,
    localhost_only,
    eclipse_zenoh_identifier);
}

/// SHUTDOWN CONTEXT ===========================================================
// Shutdown the middleware for a given context.
//
// In this case, closes the associated Zenoh session.
rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  return rmw_zenoh_common_shutdown(context, eclipse_zenoh_identifier);
}

/// INIT OPTIONS ===============================================================
// Initialize given init_options with the default values
// and implementation specific values.
//
// rmw_init_options_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__init__options__t.html
//
// Note: You should call rmw_get_zero_initialized_init_options()
// to get a zero initialized rmw_init_options_t struct first
rmw_ret_t
rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator)
{
  return rmw_zenoh_common_init_options_init(init_options, allocator, eclipse_zenoh_identifier);
}

/// FINALIZE CONTEXT ===========================================================
// Finalize a context. (Cleanup and deallocation.)
rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  return rmw_zenoh_common_context_fini(context, eclipse_zenoh_identifier);
}

/// DESTROY NODE ===============================================================
// Finalize a given node handle, reclaim the resources, and deallocate the node handle.
rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  return rmw_zenoh_common_destroy_node(node, eclipse_zenoh_identifier);
}

/// COPY OPTIONS ===============================================================
// Copy the given source init options to the destination init options.
rmw_ret_t
rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  return rmw_zenoh_common_init_options_copy(src, dst, eclipse_zenoh_identifier);
}

/// FINALIZE OPTIONS ===========================================================
// Finalize the given init_options. (Cleanup and deallocation.)
rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  return rmw_zenoh_common_init_options_fini(init_options, eclipse_zenoh_identifier);
}

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
  return rmw_zenoh_common_create_subscription(
    node,
    type_supports,
    topic_name,
    qos_profile,
    subscription_options,
    eclipse_zenoh_identifier);
}

/// DESTROY SUBSCRIPTION =======================================================
// Destroy and deallocate an RMW subscription
rmw_ret_t
rmw_destroy_subscription(
  rmw_node_t * node,
  rmw_subscription_t * subscription)
{
  return rmw_zenoh_common_destroy_subscription(
    node,
    subscription,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos_profile)
{
  return rmw_zenoh_common_subscription_get_actual_qos(
    subscription,
    qos_profile,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * count)
{
  return rmw_zenoh_common_subscription_count_matched_publishers(
    subscription,
    count,
    eclipse_zenoh_identifier);
}

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
  return rmw_zenoh_common_create_publisher(
    node,
    type_supports,
    topic_name,
    qos_profile,
    publisher_options,
    eclipse_zenoh_identifier);
}

/// DESTROY PUBLISHER ==========================================================
// Destroy and deallocate an rmw publisher.
rmw_ret_t
rmw_destroy_publisher(
  rmw_node_t * node,
  rmw_publisher_t * publisher)
{
  return rmw_zenoh_common_destroy_publisher(
    node,
    publisher,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos_profile)
{
  return rmw_zenoh_common_publisher_get_actual_qos(
    publisher,
    qos_profile,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_publisher_event_init(
  rmw_event_t * event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type)
{
  return rmw_zenoh_common_publisher_event_init(
    event,
    publisher,
    event_type,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  return rmw_zenoh_common_publish(
    publisher,
    ros_message,
    allocation,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_zenoh_common_take(
    subscription,
    ros_message,
    taken,
    allocation,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_subscription_event_init(
  rmw_event_t * event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type)
{
  return rmw_zenoh_common_subscription_event_init(
    event,
    subscription,
    event_type,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_zenoh_common_take_with_info(
    subscription,
    ros_message,
    taken,
    message_info,
    allocation,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_zenoh_common_take_loaned_message_with_info(
    subscription,
    loaned_message,
    taken,
    message_info,
    allocation);
}

rmw_client_t *
rmw_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  return rmw_zenoh_common_create_client(
    node,
    type_supports,
    service_name,
    qos_profile,
    eclipse_zenoh_identifier);
}

rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  return rmw_zenoh_common_create_service(
    node,
    type_supports,
    service_name,
    qos_profile,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * result)
{
  return rmw_zenoh_common_service_server_is_available(
    node,
    client,
    result,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_get_gid_for_publisher(
  const rmw_publisher_t * publisher,
  rmw_gid_t * gid)
{
  return rmw_zenoh_common_get_gid_for_publisher(
    publisher,
    gid,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_destroy_wait_set(
  rmw_wait_set_t * wait_set)
{
  return rmw_zenoh_common_destroy_wait_set(
    wait_set,
    eclipse_zenoh_identifier);
}

rmw_wait_set_t *
rmw_create_wait_set(
  rmw_context_t * context,
  size_t max_conditions)
{
  return rmw_zenoh_common_create_wait_set(
    context,
    max_conditions,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_trigger_guard_condition(
  const rmw_guard_condition_t * guard_condition_handle)
{
  return rmw_zenoh_common_trigger_guard_condition(
    guard_condition_handle,
    eclipse_zenoh_identifier);
}

rmw_guard_condition_t *
rmw_create_guard_condition(
  rmw_context_t * context)
{
  return rmw_zenoh_common_create_guard_condition(
    context,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header,
  void * ros_response)
{
  return rmw_zenoh_common_send_response(
    service,
    request_header,
    ros_response,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_take_request(
  const rmw_service_t * service,
  rmw_service_info_t * request_header,
  void * ros_request,
  bool * taken)
{
  return rmw_zenoh_common_take_request(
    service,
    request_header,
    ros_request,
    taken,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_destroy_service(
  rmw_node_t * node,
  rmw_service_t * service)
{
  return rmw_zenoh_common_destroy_service(
    node,
    service,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_take_response(
  const rmw_client_t * client,
  rmw_service_info_t * request_header,
  void * ros_response,
  bool * taken)
{
  return rmw_zenoh_common_take_response(
    client,
    request_header,
    ros_response,
    taken,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_send_request(
  const rmw_client_t * client,
  const void * ros_request,
  int64_t * sequence_id)
{
  return rmw_zenoh_common_send_request(
    client,
    ros_request,
    sequence_id,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_destroy_client(
  rmw_node_t * node,
  rmw_client_t * client)
{
  return rmw_zenoh_common_destroy_client(
    node,
    client,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message)
{
  return rmw_zenoh_common_return_loaned_message_from_subscription(
    subscription,
    loaned_message);
}
