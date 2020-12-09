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

#endif  // RMW_ZENOH_COMMON_CPP__RMW_ZENOH_COMMON_H_
