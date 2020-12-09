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

// Doc: http://docs.ros2.org/latest/api/rmw/init__options_8h.html

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/init_options.h"
#include "rmw/rmw.h"

#include "rcutils/logging_macros.h"
#include "rcutils/get_env.h"
#include "rcutils/strdup.h"

#include "rmw_zenoh_common_cpp/rmw_init_options_impl.hpp"

// Helper case-insensitive string comparison function
int strcicmp(char const * a, char const * b)
{
  for (;; a++, b++) {
    int d = tolower((unsigned char)*a) - tolower((unsigned char)*b);
    if (d != 0 || !*a) {
      return d;
    }
  }
}

extern "C"
{
/// INIT OPTIONS ===============================================================
// Initialize given init_options with the default values
// and implementation specific values.
//
// rmw_init_options_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__init__options__t.html
//
// Note: You should call rmw_get_zero_initialized_init_options()
// to get a zero initialized rmw_init_options_t struct first
rmw_ret_t
rmw_zenoh_common_init_options_init(
  rmw_init_options_t * init_options, rcutils_allocator_t allocator,
  const char * const eclipse_zenoh_identifier)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "rmw_init_options_init");

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

  // NOTE(CH3): For DDS only
  // init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;

  // NOTE(CH3): No security yet
  // Include a "do-nothing" enclave for now
  init_options->enclave = rcutils_strdup("/", allocator);

  if (!init_options->enclave) {
    RMW_SET_ERROR_MSG("failed to allocate options enclave");
    return RMW_RET_BAD_ALLOC;
  }

  // init_options->security_options = rmw_get_default_security_options();

  // POPULATE ZENOH SPECIFIC OPTIONS ===========================================
  // Allocate options implementation
  init_options->impl = static_cast<rmw_init_options_impl_t *>(
    allocator.allocate(sizeof(rmw_init_options_impl_t), allocator.state));
  if (!init_options->impl) {
    RMW_SET_ERROR_MSG("failed to allocate init_options impl");
    allocator.deallocate(init_options->enclave, allocator.state);
    return RMW_RET_BAD_ALLOC;
  }

  // Populate Zenoh session locator
  const char * zenoh_session_env_value;
  if (nullptr != rcutils_get_env("RMW_ZENOH_SESSION_LOCATOR", &zenoh_session_env_value)) {
    RMW_SET_ERROR_MSG("error trying to retrieve RMW_ZENOH_SESSION_LOCATOR env var");
    return RMW_RET_ERROR;
  }

  if (zenoh_session_env_value[0] == '\0') {
    init_options->impl->session_locator = nullptr;
  } else {
    init_options->impl->session_locator = rcutils_strdup(zenoh_session_env_value, allocator);
    if (!init_options->impl->session_locator) {
      RMW_SET_ERROR_MSG("failed to allocate RMW_ZENOH_SESSION_LOCATOR");
      allocator.deallocate(init_options->impl, allocator.state);
      allocator.deallocate(init_options->enclave, allocator.state);
      return RMW_RET_BAD_ALLOC;
    }
  }

  // Populate Zenoh session mode
  const char * zenoh_mode_env_value;
  if (nullptr != rcutils_get_env("RMW_ZENOH_MODE", &zenoh_mode_env_value)) {
    RMW_SET_ERROR_MSG("error trying to retrieve RMW_ZENOH_SESSION_LOCATOR env var");
    return RMW_RET_ERROR;
  }

  // Case insensitive comparisons
  if (strcicmp(zenoh_mode_env_value, "CLIENT") == 0) {
    init_options->impl->mode = rcutils_strdup("CLIENT", allocator);
  } else if (strcicmp(zenoh_mode_env_value, "ROUTER") == 0) {
    init_options->impl->mode = rcutils_strdup("ROUTER", allocator);
  } else {
    init_options->impl->mode = rcutils_strdup("PEER", allocator);
  }

  if (!init_options->impl->mode) {
    RMW_SET_ERROR_MSG("failed to allocate RMW_ZENOH_MODE");
    allocator.deallocate(init_options->impl->session_locator, allocator.state);
    allocator.deallocate(init_options->impl, allocator.state);
    allocator.deallocate(init_options->enclave, allocator.state);
    return RMW_RET_BAD_ALLOC;
  }

  return RMW_RET_OK;
}

/// COPY OPTIONS ===============================================================
// Copy the given source init options to the destination init options.
rmw_ret_t
rmw_zenoh_common_init_options_copy(
  const rmw_init_options_t * src, rmw_init_options_t * dst,
  const char * const eclipse_zenoh_identifier)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "rmw_init_options_copy");

  RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);
  if (nullptr == src->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected initialized src");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (nullptr != dst->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized dst");
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    src,
    src->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  const rcutils_allocator_t allocator = src->allocator;
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);

  // Deep copy dynamically allocated strings
  //
  // NOTE(CH3): Unfortunately we have to make do with these ugly copy pasted calls until
  // rcpputils::make_scope_exit() gets ported into a ROS2 release
  //
  // TODO(CH3): Once it is, replace the repeated deallocate calls with a scope exit handler
  rmw_init_options_t tmp = *src;  // Copy struct

  tmp.enclave = rcutils_strdup(src->enclave, allocator);
  if (nullptr != src->enclave && nullptr == tmp.enclave) {
    RMW_SET_ERROR_MSG("failed to allocate options enclave");
    return RMW_RET_BAD_ALLOC;
  }

  tmp.impl = static_cast<rmw_init_options_impl_t *>(  // Allocate new impl member for tmp
    allocator.allocate(sizeof(rmw_init_options_impl_t), allocator.state));
  if (nullptr != src->impl && nullptr == tmp.impl) {
    RMW_SET_ERROR_MSG("failed to allocate options impl");
    allocator.deallocate(tmp.enclave, allocator.state);
    return RMW_RET_BAD_ALLOC;
  }

  tmp.impl->session_locator = rcutils_strdup(src->impl->session_locator, allocator);
  if (nullptr != src->impl->session_locator && nullptr == tmp.impl->session_locator) {
    RMW_SET_ERROR_MSG("failed to allocate RMW_ZENOH_SESSION_LOCATOR");
    allocator.deallocate(tmp.impl, allocator.state);
    allocator.deallocate(tmp.enclave, allocator.state);
    return RMW_RET_BAD_ALLOC;
  }

  tmp.impl->mode = rcutils_strdup(src->impl->mode, allocator);
  if (nullptr != src->impl->mode && nullptr == tmp.impl->mode) {
    RMW_SET_ERROR_MSG("failed to allocate RMW_ZENOH_MODE");
    allocator.deallocate(tmp.impl->session_locator, allocator.state);
    allocator.deallocate(tmp.impl, allocator.state);
    allocator.deallocate(tmp.enclave, allocator.state);
    return RMW_RET_BAD_ALLOC;
  }

  // NOTE(CH3): No security yet
  // tmp.security_options = rmw_get_zero_initialized_security_options();
  // rmw_ret_t ret =
  //   rmw_security_options_copy(&src->security_options, allocator, &tmp.security_options);
  // if (RMW_RET_OK != ret) {
  //   allocator.deallocate(tmp.enclave, allocator.state);
  //   return ret;
  // }

  *dst = tmp;
  return RMW_RET_OK;
}

/// FINALIZE OPTIONS ===========================================================
// Finalize the given init_options. (Cleanup and deallocation.)
rmw_ret_t
rmw_zenoh_common_init_options_fini(
  rmw_init_options_t * init_options,
  const char * const eclipse_zenoh_identifier)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "rmw_init_options_fini");

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

  rcutils_allocator_t allocator = init_options->allocator;
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);

  allocator.deallocate(init_options->impl->session_locator, allocator.state);
  allocator.deallocate(init_options->impl->mode, allocator.state);
  allocator.deallocate(init_options->impl, allocator.state);
  allocator.deallocate(init_options->enclave, allocator.state);

  *init_options = rmw_get_zero_initialized_init_options();

  // return ret;
  return RMW_RET_OK;
}
}  // extern "C"
