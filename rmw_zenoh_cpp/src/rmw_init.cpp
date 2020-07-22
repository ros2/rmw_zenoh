// Doc: http://docs.ros2.org/latest/api/rmw/init_8h.html

#include <cstring>

#include "rcutils/logging_macros.h"

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/init.h"

#include "rmw_zenoh_cpp/identifier.hpp"
#include "rmw_zenoh_cpp/rmw_context_impl.hpp"
#include "rmw_zenoh_cpp/rmw_init_options_impl.hpp"

extern "C"
{
#include "zenoh/zenoh-ffi.h"

/// INIT CONTEXT ===============================================================
// Initialize the middleware with the given options, and yielding an context.
//
// Starts a new Zenoh session and configures it according to the init options
// with the following environment variables:
// RMW_ZENOH_SESSION_LOCATOR: Session TCP locator to use
// RMW_ZENOH_USE_CLIENT_MODE: If empty or 0, uses PEER_MODE, otherwise,
//                            uses CLIENT_MODE
rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_init");

  // CLEANUP DEFINITIONS =======================================================
  // Store a pointer to the context with an exit handler that zero inits the
  // context if any initialization steps fail
  std::unique_ptr<rmw_context_t, void (*)(rmw_context_t *)> clean_when_fail(
    context,
    [](rmw_context_t * context)
    {
      *context = rmw_get_zero_initialized_context();
    });
  rmw_ret_t ret = RMW_RET_OK;

  // ASSERTIONS ================================================================
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options,
    options->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  if (nullptr != context->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected a zero-initialized context");
    return RMW_RET_INVALID_ARGUMENT;
  }

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
  std::unique_ptr<rmw_context_impl_t> context_impl(new (std::nothrow) rmw_context_impl_t());
  if (!context_impl) {
    RMW_SET_ERROR_MSG("failed to allocate context impl");
    return RMW_RET_BAD_ALLOC;
  }

  // Open configured Zenoh session, then assign it to the context
  int SESSION_MODE;

  if (strcmp(context->options.impl->mode, "CLIENT") == 0) {
    SESSION_MODE = CLIENT_MODE;
  } else if (strcmp(context->options.impl->mode, "CLIENT") == 0) {
    SESSION_MODE = ROUTER_MODE;
  } else {
    SESSION_MODE = PEER_MODE;
  }

  ZNSession * s = zn_open(SESSION_MODE,
                          context->options.impl->session_locator,
                          0);

  if (s == nullptr) {
    RMW_SET_ERROR_MSG("failed to create Zenoh session when starting context");
    return RMW_RET_ERROR;
  } else {
    context_impl->session = s;
  }

  // CLEANUP IF PASSED =========================================================
  context->impl = context_impl.release();
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
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_shutdown");

  // Assertions
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // Close Zenoh session
  zn_close(context->impl->session);
  return RMW_RET_OK;
}

/// FINALIZE CONTEXT ===========================================================
// Finalize a context.
//
// NOTE(CH3): It looks like the standard use of this function is for checking
// and then resetting contexts to zero?? The docs say it is for finalizing.
//
// I don't understand why, so I just commented out the zeroing here..
rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_context_fini");

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rcutils_allocator_t * allocator = &context->options.allocator;

  // Deallocate implementation specific members
  allocator->deallocate(context->impl->session, allocator->state);
  delete context->impl;

  // Reset context
  *context = rmw_get_zero_initialized_context();

  return RMW_RET_OK;
}

} // extern "C"
