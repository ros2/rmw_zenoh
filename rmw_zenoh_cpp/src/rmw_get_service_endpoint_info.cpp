// Copyright 2025 Minju Lee (이민주).
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

#include "detail/identifier.hpp"
#include "detail/liveliness_utils.hpp"
#include "detail/rmw_context_impl_s.hpp"

#include "rcutils/allocator.h"

#include "rmw/get_service_endpoint_info.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/types.h"

extern "C"
{
///=============================================================================
/// Retrieve endpoint information for each known client of a given service.
rmw_ret_t
rmw_get_clients_info_by_service(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * service_name,
  bool no_mangle,
  rmw_service_endpoint_info_array_t * clients_info)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(node->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);
  if (RMW_RET_OK != rmw_service_endpoint_info_array_check_zero(clients_info)) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  return context_impl->graph_cache()->get_entities_info_by_service(
    rmw_zenoh_cpp::liveliness::EntityType::Client,
    allocator,
    service_name,
    no_mangle,
    clients_info);
}

///=============================================================================
/// Retrieve endpoint information for each known server of a given service.
rmw_ret_t
rmw_get_servers_info_by_service(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * service_name,
  bool no_mangle,
  rmw_service_endpoint_info_array_t * servers_info)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(node->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);
  if (RMW_RET_OK != rmw_service_endpoint_info_array_check_zero(servers_info)) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  return context_impl->graph_cache()->get_entities_info_by_service(
    rmw_zenoh_cpp::liveliness::EntityType::Service,
    allocator,
    service_name,
    no_mangle,
    servers_info);
}
}  // extern "C"
