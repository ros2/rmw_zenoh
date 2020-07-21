// Doc: http://docs.ros2.org/latest/api/rmw/init__options_8h.html

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/init_options.h"
#include "rmw/rmw.h"

#include "rcutils/logging_macros.h"
#include "rcutils/get_env.h"

#include "rmw_zenoh_cpp/rmw_init_options_impl.hpp"
#include "rmw_zenoh_cpp/identifier.hpp"

extern "C"
{
/// INIT OPTIONS ===============================================================
// Initialize given init_options with the default values
// and implementation specific values.
//
// Note: You should call rmw_get_zero_initialized_init_options()
// to get a zero initialized rmw_init_options_t struct first
rmw_ret_t
rmw_init_options_init(rmw_init_options_t * init_options,
                      rcutils_allocator_t allocator)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);

  if (NULL != init_options->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Populate common members
  init_options->instance_id = 0;
  init_options->implementation_identifier = eclipse_zenoh_identifier;
  init_options->allocator = allocator;
  init_options->localhost_only = RMW_LOCALHOST_ONLY_DEFAULT;

  // Populate Zenoh specific options
  init_options->impl = new rmw_init_options_impl_t();

  const char * session_env_value;
  rcutils_get_env("RMW_ZENOH_SESSION_LOCATOR", &session_env_value);

  if (strcmp(session_env_value, "") == 0) {  // if session_env_value == ""
    init_options->impl->session_locator = NULL;
  } else {
    init_options->impl->session_locator = session_env_value;
  }

  const char * client_mode_env_value;
  rcutils_get_env("RMW_ZENOH_USE_CLIENT_MODE", &client_mode_env_value);

  if (strcmp(session_env_value, "") == 0
      or strcmp(session_env_value, "0") == 0) {
    init_options->impl->client_mode = false;
  } else {
    init_options->impl->client_mode = true;
  }

  // NOTE(CH3): For DDS only
  // init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;

  // NOTE(CH3): No security yet
  // init_options->enclave = NULL;
  // init_options->security_options = rmw_get_default_security_options();

  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_init_options_init");
  return RMW_RET_OK;
}

/// COPY OPTIONS ===============================================================
// Copy the given source init options to the destination init options.
rmw_ret_t
rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    src,
    src->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (NULL != dst->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized dst");
    return RMW_RET_INVALID_ARGUMENT;
  }

  *dst = *src;
  return RMW_RET_OK;
}

/// FINALIZE OPTIONS ===========================================================
// Finalize the given init_options.
//
// NOTE(CH3): It looks like the standard use of this function is for checking
// and then resetting init options to zero?? The docs say it is for finalizing.
//
// I don't understand why, so I just commented out the zero init here..
rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  assert(eclipse_zenoh_identifier != NULL);
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  if (NULL == init_options->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init_options,
    init_options->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  rcutils_allocator_t * allocator = &init_options->allocator;
  RCUTILS_CHECK_ALLOCATOR(allocator, return RMW_RET_INVALID_ARGUMENT);

  // NOTE(CH3): No security yet
  // allocator->deallocate(init_options->enclave, allocator->state);
  // rmw_ret_t ret = rmw_security_options_fini(&init_options->security_options,
  //                                           allocator);

  // NOTE(CH3): This line seems to be in a lot of existing RMW implementations
  // but it looks like it just resets the init options?? What's up with that?
  // *init_options = rmw_get_zero_initialized_init_options();

  // return ret;
  return RMW_RET_OK;
}

} // extern "C"
