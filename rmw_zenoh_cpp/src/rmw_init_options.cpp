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

#include "detail/identifier.hpp"
#include "detail/rmw_init_options_impl.hpp"


#include "rcutils/env.h"
#include "rcutils/strdup.h"
#include "rcutils/types.h"

#include "rmw/init.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/init_options.h"

#include "zenoh.h"

extern "C"
{
//==============================================================================
/// Initialize given init options with the default values and implementation specific values.
rmw_ret_t
rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);
  if (NULL != init_options->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected zero-initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }

  init_options->instance_id = 0;
  init_options->implementation_identifier = rmw_zenoh_identifier;
  init_options->allocator = allocator;
  init_options->impl = nullptr;
  init_options->enclave = NULL;
  init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;
  init_options->security_options = rmw_get_default_security_options();
  init_options->localhost_only = RMW_LOCALHOST_ONLY_DEFAULT;
  init_options->discovery_options = rmw_get_zero_initialized_discovery_options();
  return rmw_discovery_options_init(&(init_options->discovery_options), 0, &allocator);

  // Initialize impl.
  init_options->impl = static_cast<rmw_init_options_impl_s *>(
    allocator.allocate(sizeof(rmw_init_options_impl_s), allocator.state));
  if (!init_options->impl) {
    RMW_SET_ERROR_MSG("failed to allocate init_options impl");
    allocator.deallocate(init_options->enclave, allocator.state);
    return RMW_RET_BAD_ALLOC;
  }

  // Initialize the zenoh config. We use the default config if a path to the
  // config is unavailable.
  z_owned_config_t config;
  const char * zenoh_config_path;
  if (nullptr != rcutils_get_env("ZENOH_CONFIG_PATH", &zenoh_config_path)) {
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
  init_options->impl->config = z_config_loan(&config);
  init_options->impl->session = z_open(z_move(config));
  if (!z_session_check(&init_options->impl->session)) {
    RMW_SET_ERROR_MSG("Error setting up zenoh session");
    return RMW_RET_INVALID_ARGUMENT;
  }

  return RMW_RET_OK;
}

//==============================================================================
/// Copy the given source init options to the destination init options.
rmw_ret_t
rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Finalize the given init options.
rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  if (NULL == init_options->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init_options,
    init_options->implementation_identifier,
    rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  rcutils_allocator_t * allocator = &init_options->allocator;
  RCUTILS_CHECK_ALLOCATOR(allocator, return RMW_RET_INVALID_ARGUMENT);

  allocator->deallocate(init_options->enclave, allocator->state);
  rmw_ret_t ret = rmw_security_options_fini(&init_options->security_options, allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  // Close the zenoh session and deallocate the impl.
  rmw_init_options_impl_s * impl = static_cast<rmw_init_options_impl_s *>(init_options->impl);
  if (nullptr == impl) {
    RMW_SET_ERROR_MSG("invalid impl in init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }
  z_close(z_move(impl->session));
  allocator->deallocate(init_options->impl, allocator->state);

  ret = rmw_discovery_options_fini(&init_options->discovery_options);
  *init_options = rmw_get_zero_initialized_init_options();

  return ret;
}

} // extern "C"
