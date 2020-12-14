// Copyright 2020 ADLINK, Inc.
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

#include <array>
#include <utility>
#include <set>
#include <string>

#include "rcutils/filesystem.h"
#include "rcutils/logging_macros.h"

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"

#include "rmw_zenoh_common_cpp/init_rmw_context_impl.hpp"
#include "rmw_zenoh_common_cpp/rmw_common.hpp"
#include "rmw_zenoh_common_cpp/rmw_context_impl.hpp"

#include "rmw_zenoh_dynamic_cpp/identifier.hpp"
#include "rmw_zenoh_dynamic_cpp/init_rmw_context_impl.hpp"

extern "C"
{
rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_,
  size_t domain_id,
  bool localhost_only)
{
  (void)domain_id;
  (void)localhost_only;
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init context,
    context->implementation_identifier,
    eclipse_zenoh_identifier,
    // TODO(wjwwood): replace this with RMW_RET_INCORRECT_RMW_IMPLEMENTATION when refactored
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return nullptr);
  if (context->impl->is_shutdown) {
    RCUTILS_SET_ERROR_MSG("context has been shutdown");
    return nullptr;
  }

  if (RMW_RET_OK != rmw_zenoh_dynamic_cpp::increment_context_impl_ref_count(context)) {
    return nullptr;
  }

  rmw_node_t * node = rmw_zenoh_common_cpp::__rmw_create_node(
    context, eclipse_zenoh_identifier, name, namespace_);

  if (nullptr == node) {
    if (RMW_RET_OK != rmw_zenoh_common_cpp::decrement_context_impl_ref_count(context)) {
      RCUTILS_SAFE_FWRITE_TO_STDERR(
        "'decrement_context_impl_ref_count' failed while being executed due to '"
        RCUTILS_STRINGIFY(__function__) "' failing");
    }
  }
  return node;
}

rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  rmw_context_t * context = node->context;
  rmw_ret_t ret = rmw_zenoh_common_cpp::__rmw_destroy_node(
    eclipse_zenoh_identifier, node);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  return rmw_zenoh_common_cpp::decrement_context_impl_ref_count(context);
}

const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(const rmw_node_t * node)
{
  return rmw_zenoh_common_cpp::__rmw_node_get_graph_guard_condition(
    node);
}
}  // extern "C"
