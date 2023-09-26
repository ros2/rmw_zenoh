// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <zenoh.h>

#include <new>

#include "detail/guard_condition.hpp"
#include "detail/identifier.hpp"
#include "detail/rmw_data_types.hpp"

#include "rcutils/env.h"
#include "rcutils/strdup.h"
#include "rcutils/types.h"

#include "rmw/init.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/init_options.h"

#include "rcpputils/scope_exit.hpp"

extern "C"
{
//==============================================================================
/// Initialize the middleware with the given options, and yielding an context.
rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  fprintf(stderr, "Initializing context\n");
  RMW_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->implementation_identifier,
    "expected initialized init options",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options,
    options->implementation_identifier,
    rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->enclave,
    "expected non-null enclave",
    return RMW_RET_INVALID_ARGUMENT);
  if (NULL != context->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected a zero-initialized context");
    return RMW_RET_INVALID_ARGUMENT;
  }

  auto restore_context = rcpputils::make_scope_exit(
    [context]() {*context = rmw_get_zero_initialized_context();});

  context->instance_id = options->instance_id;
  context->implementation_identifier = rmw_zenoh_identifier;
  // No custom handling of RMW_DEFAULT_DOMAIN_ID. Simply use a reasonable domain id.
  context->actual_domain_id =
    RMW_DEFAULT_DOMAIN_ID != options->domain_id ? options->domain_id : 0u;

  const rcutils_allocator_t * allocator = &options->allocator;

  context->impl = static_cast<rmw_context_impl_t *>(
    allocator->zero_allocate(1, sizeof(rmw_context_impl_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "failed to allocate context impl",
    return RMW_RET_BAD_ALLOC);
  auto free_impl = rcpputils::make_scope_exit(
    [context, allocator]() {
      allocator->deallocate(context->impl, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(context->impl, context->impl, return RMW_RET_BAD_ALLOC, rmw_context_impl_t);
  auto impl_destructor = rcpputils::make_scope_exit(
    [context] {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        context->impl->~rmw_context_impl_t(), rmw_context_impl_t);
    });

  rmw_ret_t ret;
  if ((ret = rmw_init_options_copy(options, &context->options)) != RMW_RET_OK) {
    // error already set
    return ret;
  }
  auto free_options = rcpputils::make_scope_exit(
    [context]() {
      rmw_ret_t ret = rmw_init_options_fini(&context->options);
      if (ret != RMW_RET_OK) {
        RMW_SAFE_FWRITE_TO_STDERR("Failed to cleanup context options during error handling");
      }
    });

  // Initialize context's implementation
  context->impl->is_shutdown = false;

  // Initialize the zenoh config. We use the default config if a path to the
  // config is unavailable.
  z_owned_config_t config;
  const char * zenoh_config_path;
  if (NULL != rcutils_get_env("ZENOH_CONFIG_PATH", &zenoh_config_path)) {
    RMW_SET_ERROR_MSG("Error in reading ZENOH_CONFIG_PATH envar");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (zenoh_config_path[0] == '\0') {
    // No config path set.
    config = z_config_default();
  } else {
    config = zc_config_from_file(zenoh_config_path);
    if (!z_config_check(&config)) {
      RMW_SET_ERROR_MSG("Error in zenoh config path");
      return RMW_RET_INVALID_ARGUMENT;
    }
  }

  // Initialize the zenoh session.
  context->impl->session = z_open(z_move(config));
  if (!z_session_check(&context->impl->session)) {
    RMW_SET_ERROR_MSG("Error setting up zenoh session");
    return RMW_RET_INVALID_ARGUMENT;
  }
  auto close_session = rcpputils::make_scope_exit(
    [context]() {
      z_close(z_move(context->impl->session));
    });

  // Initialize the guard condition.
  context->impl->graph_guard_condition =
    static_cast<rmw_guard_condition_t *>(allocator->zero_allocate(
      1, sizeof(rmw_guard_condition_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl->graph_guard_condition,
    "failed to allocate graph guard condition",
    return RMW_RET_BAD_ALLOC);
  auto free_guard_condition = rcpputils::make_scope_exit(
    [context, allocator]() {
      allocator->deallocate(context->impl->graph_guard_condition, allocator->state);
    });

  context->impl->graph_guard_condition->implementation_identifier = rmw_zenoh_identifier;

  context->impl->graph_guard_condition->data =
    allocator->zero_allocate(1, sizeof(GuardCondition), allocator->state);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl->graph_guard_condition->data,
    "failed to allocate graph guard condition data",
    return RMW_RET_BAD_ALLOC);
  auto free_guard_condition_data = rcpputils::make_scope_exit(
    [context, allocator]() {
      allocator->deallocate(context->impl->graph_guard_condition->data, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(
    context->impl->graph_guard_condition->data,
    context->impl->graph_guard_condition->data,
    return RMW_RET_BAD_ALLOC,
    GuardCondition);
  auto destruct_guard_condition_data = rcpputils::make_scope_exit(
    [context]() {
      auto gc_data = static_cast<GuardCondition *>(context->impl->graph_guard_condition->data);
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(gc_data->~GuardCondition(), GuardCondition);
    });

  destruct_guard_condition_data.cancel();
  free_guard_condition_data.cancel();
  free_guard_condition.cancel();
  close_session.cancel();
  free_options.cancel();
  impl_destructor.cancel();
  free_impl.cancel();
  restore_context.cancel();

  return RMW_RET_OK;
}

//==============================================================================
/// Shutdown the middleware for a given context.
rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  fprintf(stderr, "Shutting down context\n");
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // Close the zenoh session
  if (z_close(z_move(context->impl->session)) < 0) {
    RMW_SET_ERROR_MSG("Error while closing zenoh session");
    return RMW_RET_ERROR;
  }
  context->impl->is_shutdown = true;
  return RMW_RET_OK;
}

//==============================================================================
/// Finalize a context.
rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  fprintf(stderr, "Finalizing context\n");
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (!context->impl->is_shutdown) {
    RCUTILS_SET_ERROR_MSG("context has not been shutdown");
    return RMW_RET_INVALID_ARGUMENT;
  }

  const rcutils_allocator_t * allocator = &context->options.allocator;

  RMW_TRY_DESTRUCTOR(
    static_cast<GuardCondition *>(context->impl->graph_guard_condition->data)->~GuardCondition(),
    GuardCondition, );
  allocator->deallocate(context->impl->graph_guard_condition->data, allocator->state);

  allocator->deallocate(context->impl->graph_guard_condition, allocator->state);

  RMW_TRY_DESTRUCTOR(context->impl->~rmw_context_impl_t(), rmw_context_impl_t, );
  allocator->deallocate(context->impl, allocator->state);

  rmw_ret_t ret = rmw_init_options_fini(&context->options);

  *context = rmw_get_zero_initialized_context();

  return ret;
}
}  // extern "C"
