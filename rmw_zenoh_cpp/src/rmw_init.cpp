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
#include "detail/rmw_data_types.hpp"
#include "detail/zenoh_config.hpp"

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

namespace
{
void
graph_sub_data_handler(const z_sample_t * sample, void * data)
{
  static_cast<void>(data);

  z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
  auto free_keystr = rcpputils::make_scope_exit(
    [&keystr]() {
      z_drop(z_move(keystr));
    });

  // Get the context impl from data.
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(
    data);
  if (context_impl == nullptr) {
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "[graph_sub_data_handler] Unable to convert data into context_impl"
    );
    return;
  }

  switch (sample->kind) {
    case z_sample_kind_t::Z_SAMPLE_KIND_PUT:
      context_impl->graph_cache->parse_put(keystr._cstr);
      break;
    case z_sample_kind_t::Z_SAMPLE_KIND_DELETE:
      context_impl->graph_cache->parse_del(keystr._cstr);
      break;
    default:
      return;
  }

  rmw_ret_t rmw_ret = rmw_trigger_guard_condition(context_impl->graph_guard_condition);
  if (RMW_RET_OK != rmw_ret) {
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "[graph_sub_data_handler] Unable to trigger graph guard condition"
    );
  }
}
}  // namespace

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
        context->impl->~rmw_context_impl_t(), rmw_context_impl_t *);
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

  // Set the enclave.
  context->impl->enclave = rcutils_strdup(options->enclave, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl->enclave,
    "failed to allocate enclave",
    return RMW_RET_BAD_ALLOC);
  auto free_enclave = rcpputils::make_scope_exit(
    [context, allocator]() {
      allocator->deallocate(context->impl->enclave, allocator->state);
    });

  // Initialize context's implementation
  context->impl->is_shutdown = false;

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
  auto free_shm_enabled = rcpputils::make_scope_exit(
    [&shm_enabled]() {
      z_drop(z_move(shm_enabled));
    });

  // Initialize the zenoh session.
  context->impl->session = z_open(z_move(config));
  if (!z_session_check(&context->impl->session)) {
    RMW_SET_ERROR_MSG("Error setting up zenoh session");
    return RMW_RET_ERROR;
  }
  auto close_session = rcpputils::make_scope_exit(
    [context]() {
      z_close(z_move(context->impl->session));
    });

  /// Initialize the graph cache.
  z_id_t zid = z_info_zid(z_loan(context->impl->session));
  context->impl->graph_cache = std::make_unique<rmw_zenoh_cpp::GraphCache>(zid);

  // Check if a zenoh router is available.
  context->impl->async_check_router();

  // Initialize the shm manager if shared_memory is enabled in the config.
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
    context->impl->shm_manager =
      zc_shm_manager_new(
      z_loan(context->impl->session),
      idstr,
      SHM_BUFFER_SIZE_MB * 1024 * 1024);
    if (!context->impl->shm_manager.has_value() ||
      !zc_shm_manager_check(&context->impl->shm_manager.value()))
    {
      RMW_SET_ERROR_MSG("Unable to create shm manager.");
      return RMW_RET_ERROR;
    }
  }
  auto free_shm_manager = rcpputils::make_scope_exit(
    [context]() {
      if (context->impl->shm_manager.has_value()) {
        z_drop(z_move(context->impl->shm_manager.value()));
      }
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

  context->impl->graph_guard_condition->implementation_identifier =
    rmw_zenoh_cpp::rmw_zenoh_identifier;

  context->impl->graph_guard_condition->data =
    allocator->zero_allocate(1, sizeof(rmw_zenoh_cpp::GuardCondition), allocator->state);
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
    rmw_zenoh_cpp::GuardCondition);
  auto destruct_guard_condition_data = rcpputils::make_scope_exit(
    [context]() {
      auto gc_data =
      static_cast<rmw_zenoh_cpp::GuardCondition *>(context->impl->graph_guard_condition->data);
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        gc_data->~GuardCondition(),
        rmw_zenoh_cpp::GuardCondition);
    });

  // Setup liveliness subscriptions for discovery.
  const std::string liveliness_str = rmw_zenoh_cpp::liveliness::subscription_token(
    context->actual_domain_id);

  // Query router/liveliness participants to get graph information before this session was started.

  // We create a blocking channel that is unbounded, ie. `bound` = 0, to receive
  // replies for the zc_liveliness_get() call. This is necessary as if the `bound`
  // is too low, the channel may starve the zenoh executor of its threads which
  // would lead to deadlocks when trying to receive replies and block the
  // execution here.
  // The blocking channel will return when the sender end is closed which is
  // the moment the query finishes.
  // The non-blocking fifo exists only for the use case where we don't want to
  // block the thread between responses (including the request termination response).
  // In general, unless we want to cooperatively schedule other tasks on the same
  // thread as reading the fifo, the blocking fifo will be more appropriate as
  // the code will be simpler, and if we're just going to spin over the non-blocking
  // reads until we obtain responses, we'll just be hogging CPU time by convincing
  // the OS that we're doing actual work when it could instead park the thread.
  z_owned_reply_channel_t channel = zc_reply_fifo_new(0);
  zc_liveliness_get(
    z_loan(context->impl->session), z_keyexpr(liveliness_str.c_str()),
    z_move(channel.send), NULL);
  z_owned_reply_t reply = z_reply_null();
  for (bool call_success = z_call(channel.recv, &reply); !call_success || z_check(reply);
    call_success = z_call(channel.recv, &reply))
  {
    if (!call_success) {
      continue;
    }
    if (z_reply_is_ok(&reply)) {
      z_sample_t sample = z_reply_ok(&reply);
      z_owned_str_t keystr = z_keyexpr_to_string(sample.keyexpr);
      // Ignore tokens from the same session to avoid race conditions from this
      // query and the liveliness subscription.
      context->impl->graph_cache->parse_put(z_loan(keystr), true);
      z_drop(z_move(keystr));
    } else {
      printf("[discovery] Received an error\n");
    }
  }
  z_drop(z_move(reply));
  z_drop(z_move(channel));

  // TODO(Yadunund): Switch this to a liveliness subscriptions once the API is available.

  // Uncomment and rely on #if #endif blocks to enable this feature when building with
  // zenoh-pico since liveliness is only available in zenoh-c.
  // auto sub_options = z_subscriber_options_default();
  // sub_options.reliability = Z_RELIABILITY_RELIABLE;
  // context->impl->graph_subscriber = z_declare_subscriber(
  //   z_loan(context->impl->session),
  //   z_keyexpr(liveliness_str.c_str()),
  //   z_move(callback),
  //   &sub_options);
  auto sub_options = zc_liveliness_subscriber_options_null();
  z_owned_closure_sample_t callback = z_closure(graph_sub_data_handler, nullptr, context->impl);
  context->impl->graph_subscriber = zc_liveliness_declare_subscriber(
    z_loan(context->impl->session),
    z_keyexpr(liveliness_str.c_str()),
    z_move(callback),
    &sub_options);
  zc_liveliness_subscriber_options_drop(z_move(sub_options));
  auto undeclare_z_sub = rcpputils::make_scope_exit(
    [context]() {
      z_undeclare_subscriber(z_move(context->impl->graph_subscriber));
    });
  if (!z_check(context->impl->graph_subscriber)) {
    RMW_SET_ERROR_MSG("unable to create zenoh subscription");
    return RMW_RET_ERROR;
  }

  undeclare_z_sub.cancel();
  close_session.cancel();
  destruct_guard_condition_data.cancel();
  impl_destructor.cancel();
  free_guard_condition_data.cancel();
  free_guard_condition.cancel();
  free_enclave.cancel();
  free_options.cancel();
  impl_destructor.cancel();
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

  z_undeclare_subscriber(z_move(context->impl->graph_subscriber));
  if (context->impl->shm_manager.has_value()) {
    z_drop(z_move(context->impl->shm_manager.value()));
  }
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
  if (!context->impl->is_shutdown) {
    RCUTILS_SET_ERROR_MSG("context has not been shutdown");
    return RMW_RET_INVALID_ARGUMENT;
  }

  const rcutils_allocator_t * allocator = &context->options.allocator;

  RMW_TRY_DESTRUCTOR(
    static_cast<rmw_zenoh_cpp::GuardCondition *>(
      context->impl->graph_guard_condition->data)->~GuardCondition(),
    rmw_zenoh_cpp::GuardCondition, );
  allocator->deallocate(context->impl->graph_guard_condition->data, allocator->state);

  allocator->deallocate(context->impl->graph_guard_condition, allocator->state);
  context->impl->graph_guard_condition = nullptr;

  allocator->deallocate(context->impl->enclave, allocator->state);

  RMW_TRY_DESTRUCTOR(context->impl->~rmw_context_impl_t(), rmw_context_impl_t *, );

  allocator->deallocate(context->impl, allocator->state);

  rmw_ret_t ret = rmw_init_options_fini(&context->options);

  *context = rmw_get_zero_initialized_context();

  return ret;
}
}  // extern "C"
