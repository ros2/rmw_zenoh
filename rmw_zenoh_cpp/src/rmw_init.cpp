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
#include <string>
#include <thread>

#include "detail/guard_condition.hpp"
#include "detail/identifier.hpp"
#include "detail/liveliness_utils.hpp"
#include "detail/rmw_context_impl_s.hpp"
#include "detail/rmw_data_types.hpp"
#include "detail/zenoh_config.hpp"
#include "detail/zenoh_router_check.hpp"

#include "rcutils/env.h"
#include "detail/logging_macros.hpp"
#include "rcutils/strdup.h"
#include "rcutils/types.h"

#include "rmw/init.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/init_options.h"

#include "rcpputils/scope_exit.hpp"

extern "C"
{
// Megabytes of SHM to reserve.
// TODO(clalancette): Make this configurable, or get it from the configuration
#define SHM_BUFFER_SIZE_MB 10

//==============================================================================
/// Initialize the middleware with the given options, and yielding an context.
rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->implementation_identifier,
    "expected initialized init options",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options,
    options->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
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
  context->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  // No custom handling of RMW_DEFAULT_DOMAIN_ID. Simply use a reasonable domain id.
  context->actual_domain_id =
    RMW_DEFAULT_DOMAIN_ID != options->domain_id ? options->domain_id : 0u;

  const rcutils_allocator_t * allocator = &options->allocator;

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

  // If not already defined, set the logging environment variable for Zenoh sessions
  // to warning level by default.
  // TODO(Yadunund): Switch to rcutils_get_env once it supports not overwriting values.
  if (setenv(ZENOH_LOG_ENV_VAR_STR, ZENOH_LOG_WARN_LEVEL_STR, 0) != 0) {
    RMW_SET_ERROR_MSG("Error configuring Zenoh logging.");
    return RMW_RET_ERROR;
  }

  // Initialize the zenoh configuration.
  z_owned_config_t config;
  if ((ret =
    rmw_zenoh_cpp::get_z_config(
      rmw_zenoh_cpp::ConfigurableEntity::Session,
      &config)) != RMW_RET_OK)
  {
    RMW_SET_ERROR_MSG("Error configuring Zenoh session.");
    return ret;
  }

  // Check if shm is enabled.
  z_owned_str_t shm_enabled = zc_config_get(z_loan(config), "transport/shared_memory/enabled");
  auto always_free_shm_enabled = rcpputils::make_scope_exit(
    [&shm_enabled]() {
      z_drop(z_move(shm_enabled));
    });

  // Initialize the zenoh session.
  z_owned_session_t session = z_open(z_move(config));
  if (!z_session_check(&session)) {
    RMW_SET_ERROR_MSG("Error setting up zenoh session");
    return RMW_RET_ERROR;
  }
  auto close_session = rcpputils::make_scope_exit(
    [&session]() {
      z_close(z_move(session));
    });

  // TODO(Yadunund) Move this check into a separate thread.
  // Verify if the zenoh router is running if configured.
  const std::optional<uint64_t> configured_connection_attempts =
    rmw_zenoh_cpp::zenoh_router_check_attempts();
  if (configured_connection_attempts.has_value()) {
    ret = RMW_RET_ERROR;
    uint64_t connection_attempts = 0;
    // Retry until the connection is successful.
    while (ret != RMW_RET_OK && connection_attempts < configured_connection_attempts.value()) {
      if ((ret = rmw_zenoh_cpp::zenoh_router_check(z_loan(session))) != RMW_RET_OK) {
        ++connection_attempts;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (ret != RMW_RET_OK) {
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "Unable to connect to a Zenoh router after %zu retries.",
        configured_connection_attempts.value());
      return ret;
    }
  }

  // Initialize the shm manager if shared_memory is enabled in the config.
  std::optional<zc_owned_shm_manager_t> shm_manager = std::nullopt;
  const z_id_t zid = z_info_zid(z_loan(session));
  if (shm_enabled._cstr != nullptr &&
    strcmp(shm_enabled._cstr, "true") == 0)
  {
    char idstr[sizeof(zid.id) * 2 + 1];  // 2 bytes for each byte of the id, plus the trailing \0
    static constexpr size_t max_size_of_each = 3;  // 2 for each byte, plus the trailing \0
    for (size_t i = 0; i < sizeof(zid.id); ++i) {
      snprintf(idstr + 2 * i, max_size_of_each, "%02x", zid.id[i]);
    }
    idstr[sizeof(zid.id) * 2] = '\0';
    // TODO(yadunund): Can we get the size of the shm from the config even though it's not
    // a standard parameter?
    shm_manager =
      zc_shm_manager_new(
      z_loan(session),
      idstr,
      SHM_BUFFER_SIZE_MB * 1024 * 1024);
    if (!shm_manager.has_value() ||
      !zc_shm_manager_check(&shm_manager.value()))
    {
      RMW_SET_ERROR_MSG("Unable to create shm manager.");
      return RMW_RET_ERROR;
    }
  }
  auto free_shm_manager = rcpputils::make_scope_exit(
    [&shm_manager]() {
      if (shm_manager.has_value()) {
        z_drop(z_move(shm_manager.value()));
      }
    });

  // Initialize the guard condition.
  rmw_guard_condition_t * graph_guard_condition =
    static_cast<rmw_guard_condition_t *>(allocator->zero_allocate(
      1, sizeof(rmw_guard_condition_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    graph_guard_condition,
    "failed to allocate graph guard condition",
    return RMW_RET_BAD_ALLOC);
  auto free_guard_condition = rcpputils::make_scope_exit(
    [graph_guard_condition, allocator]() {
      allocator->deallocate(graph_guard_condition, allocator->state);
    });
  graph_guard_condition->implementation_identifier =
    rmw_zenoh_cpp::rmw_zenoh_identifier;
  graph_guard_condition->data =
    allocator->zero_allocate(1, sizeof(rmw_zenoh_cpp::GuardCondition), allocator->state);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    graph_guard_condition->data,
    "failed to allocate graph guard condition data",
    return RMW_RET_BAD_ALLOC);
  auto free_guard_condition_data = rcpputils::make_scope_exit(
    [graph_guard_condition, allocator]() {
      allocator->deallocate(graph_guard_condition->data, allocator->state);
    });
  RMW_TRY_PLACEMENT_NEW(
    graph_guard_condition->data,
    graph_guard_condition->data,
    return RMW_RET_BAD_ALLOC,
    rmw_zenoh_cpp::GuardCondition);
  auto destruct_guard_condition_data = rcpputils::make_scope_exit(
    [graph_guard_condition]() {
      auto gc_data =
      static_cast<rmw_zenoh_cpp::GuardCondition *>(graph_guard_condition->data);
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        gc_data->~GuardCondition(),
        rmw_zenoh_cpp::GuardCondition);
    });


  // Create the context impl.
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

  RMW_TRY_PLACEMENT_NEW(
    context->impl,
    context->impl,
    return RMW_RET_BAD_ALLOC,
    rmw_context_impl_t,
    allocator,
    context->actual_domain_id,
    std::string(options->enclave),
    std::move(session),
    std::move(shm_manager),
    graph_guard_condition
  );

  close_session.cancel();
  destruct_guard_condition_data.cancel();
  free_guard_condition_data.cancel();
  free_guard_condition.cancel();
  free_options.cancel();
  free_impl.cancel();
  free_shm_manager.cancel();
  restore_context.cancel();

  return RMW_RET_OK;
}

//==============================================================================
/// Shutdown the middleware for a given context.
rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);

  return context_impl->shutdown();
}

//==============================================================================
/// Finalize a context.
rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (!context->impl->is_shutdown()) {
    RCUTILS_SET_ERROR_MSG("context has not been shutdown");
    return RMW_RET_INVALID_ARGUMENT;
  }

  const rcutils_allocator_t * allocator = &context->options.allocator;

  RMW_TRY_DESTRUCTOR(context->impl->~rmw_context_impl_t(), rmw_context_impl_t *, );

  allocator->deallocate(context->impl, allocator->state);

  rmw_ret_t ret = rmw_init_options_fini(&context->options);

  *context = rmw_get_zero_initialized_context();

  return ret;
}
}  // extern "C"
