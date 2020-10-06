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

#include "rmw_zenoh_cpp/identifier.hpp"
#include "rmw_zenoh_cpp/rmw_context_impl.hpp"
#include "rmw_zenoh_cpp/rmw_init_options_impl.hpp"

extern "C"
{
#include "zenoh/zenoh.h"

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

  // ASSERTIONS ================================================================
  // Check context
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  if (nullptr != context->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected a zero-initialized context");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Check options
  RMW_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->implementation_identifier,
    "expected initialized init options",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->enclave,
    "expected non-null enclave",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options,
    options->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // CLEANUP DEFINITIONS =======================================================
  // Store a pointer to the context with a custom deleter that zero inits the
  // context if any initialization steps fail
  std::unique_ptr<rmw_context_t, void (*)(rmw_context_t *)> clean_when_fail(
    context,
    [](rmw_context_t * context) {*context = rmw_get_zero_initialized_context();});

  rmw_ret_t ret = RMW_RET_OK;

  // INIT CONTEXT ==============================================================
  context->instance_id = options->instance_id;
  context->implementation_identifier = eclipse_zenoh_identifier;

  // Populate options
  context->options = rmw_get_zero_initialized_init_options();

  ret = rmw_init_options_copy(options, &context->options);
  if (RMW_RET_OK != ret) {
    if (RMW_RET_OK != rmw_init_options_fini(&context->options)) {
      RMW_SAFE_FWRITE_TO_STDERR(
        "'rmw_init_options_fini' failed while being executed due to '"
        RCUTILS_STRINGIFY(__function__) "' failing.\n");
    }
    return ret;
  }

  // Create implementation specific context
  rcutils_allocator_t * allocator = &context->options.allocator;

  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(
    allocator->allocate(sizeof(rmw_context_impl_t), allocator->state));

  if (!context_impl) {
    RMW_SET_ERROR_MSG("failed to allocate context impl");
    return RMW_RET_BAD_ALLOC;
  }

  // Open configured Zenoh session, then assign it to the context
  int SESSION_MODE;

  if (strcmp(context->options.impl->mode, "CLIENT") == 0) {
    SESSION_MODE = CLIENT;
  } else if (strcmp(context->options.impl->mode, "CLIENT") == 0) {
    SESSION_MODE = ROUTER;
  } else {
    SESSION_MODE = PEER;
  }

  ZNSession * session = zn_open(SESSION_MODE, context->options.impl->session_locator, 0);

  if (session == nullptr) {
    RMW_SET_ERROR_MSG("failed to create Zenoh session when starting context");
    allocator->deallocate(context_impl, allocator->state);
    return RMW_RET_ERROR;
  } else {
    context_impl->session = session;
    context_impl->is_shutdown = false;
  }

  // CLEANUP IF PASSED =========================================================
  context->impl = context_impl;
  clean_when_fail.release();
  return RMW_RET_OK;
}

/// SHUTDOWN CONTEXT ===========================================================
// Shutdown the middleware for a given context.
//
// In this case, closes the associated Zenoh session.
rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_shutdown");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // CLEANUP ===================================================================
  // Close Zenoh session
  if (context->impl->is_shutdown == false) {
    zn_close(context->impl->session);
    context->impl->is_shutdown = true;
  }

  return RMW_RET_OK;
}

/// FINALIZE CONTEXT ===========================================================
// Finalize a context. (Cleanup and deallocation.)
rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_context_fini");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (!context->impl->is_shutdown) {
    RMW_SET_ERROR_MSG("context has not been shutdown");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &context->options.allocator;

  // CLEANUP ===================================================================
  // Deallocate implementation specific members
  allocator->deallocate(context->impl, allocator->state);

  // Reset context
  *context = rmw_get_zero_initialized_context();

  return RMW_RET_OK;
}
}  // extern "C"
