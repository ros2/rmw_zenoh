// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <string>

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

#include "rmw/impl/cpp/macros.hpp"

#include "rmw_zenoh_common_cpp/custom_session_info.hpp"
#include "rmw_zenoh_common_cpp/custom_publisher_info.hpp"
#include "rmw_zenoh_common_cpp/publisher.hpp"
#include "rmw_zenoh_common_cpp/rmw_common.hpp"
#include "rmw_zenoh_common_cpp/rmw_context_impl.hpp"

#include "rmw_zenoh_dynamic_cpp/identifier.hpp"
#include "rmw_zenoh_dynamic_cpp/publisher.hpp"

#include "type_support_common.hpp"
#include "type_support_registry.hpp"

extern "C"
{
rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_publisher_allocation_t * allocation)
{
  // Unused in current implementation.
  (void) type_support;
  (void) message_bounds;
  (void) allocation;
  RMW_SET_ERROR_MSG("unimplemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_fini_publisher_allocation(rmw_publisher_allocation_t * allocation)
{
  // Unused in current implementation.
  (void) allocation;
  RMW_SET_ERROR_MSG("unimplemented");
  return RMW_RET_UNSUPPORTED;
}

rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies,
  const rmw_publisher_options_t * publisher_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr);

  auto session_info =
    static_cast<CustomSessionInfo *>(node->context->impl->session_info);
  rmw_publisher_t * publisher = rmw_zenoh_dynamic_cpp::create_publisher(
    session_info,
    type_supports,
    topic_name,
    qos_policies,
    publisher_options);

  if (!publisher) {
    return nullptr;
  }

  return publisher;
}

rmw_ret_t
rmw_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_count, RMW_RET_INVALID_ARGUMENT);

  return rmw_zenoh_common_cpp::__rmw_publisher_count_matched_subscriptions(
    publisher, subscription_count);
}

rmw_ret_t
rmw_publisher_assert_liveliness(const rmw_publisher_t * publisher)
{
  return rmw_zenoh_common_cpp::__rmw_publisher_assert_liveliness(
    eclipse_zenoh_identifier, publisher);
}

rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);

  return rmw_zenoh_common_cpp::__rmw_publisher_get_actual_qos(
    publisher, qos);
}

rmw_ret_t
rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message)
{
  (void) publisher;
  (void) type_support;
  (void) ros_message;

  RMW_SET_ERROR_MSG("rmw_borrow_loaned_message is not implemented for rmw_zenoh_dynamic_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_return_loaned_message_from_publisher(
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  (void) publisher;
  (void) loaned_message;

  RMW_SET_ERROR_MSG(
    "rmw_return_loaned_message_from_publisher is not implemented for rmw_zenoh_dynamic_cpp");
  return RMW_RET_UNSUPPORTED;
}

using BaseTypeSupport = rmw_zenoh_dynamic_cpp::BaseTypeSupport;

rmw_ret_t
rmw_destroy_publisher(rmw_node_t * node, rmw_publisher_t * publisher)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto info = static_cast<CustomPublisherInfo *>(publisher->data);
  auto impl = static_cast<const BaseTypeSupport *>(info->type_support_impl_);
  auto ros_type_support = static_cast<const rosidl_message_type_support_t *>(
    impl->ros_type_support());

  TypeSupportRegistry & type_registry = TypeSupportRegistry::get_instance();
  type_registry.return_message_type_support(ros_type_support);

  return rmw_zenoh_common_cpp::__rmw_destroy_publisher(
    eclipse_zenoh_identifier, node, publisher);
}
}  // extern "C"
