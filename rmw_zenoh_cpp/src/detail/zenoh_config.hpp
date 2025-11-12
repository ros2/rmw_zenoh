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

#ifndef DETAIL__ZENOH_CONFIG_HPP_
#define DETAIL__ZENOH_CONFIG_HPP_

#include <optional>
#include <unordered_map>
#include <utility>

#include <zenoh.hxx>
#include <zenoh/api/config.hxx>

#include "rmw/ret_types.h"

#define ZENOH_LOG_ENV_VAR_STR "RUST_LOG"
#define ZENOH_LOG_WARN_LEVEL_STR "warn"
#define ZENOH_LOG_INFO_LEVEL_STR "info"

namespace rmw_zenoh_cpp
{
///=============================================================================
enum class ConfigurableEntity : uint8_t
{
  Invalid = 0,
  Session,
  Router
};

///=============================================================================
/// Get the zenoh configuration for a configurable entity.
/// @details The behavior is as follows:
///   - If the environment variable for the entity is set, the z_owned_config_t
///     is configured as per the configuration file retrieved.
///   - If the environment variable is not set, the z_owned_config_t
///     is configured using the rmw_zenoh default configuration file.
/// @param entity The zenoh entity to be configured.
/// @param config The zenoh configuration to be filled.
/// @returns The zenoh configuration to be filled.
[[nodiscard]]
std::optional<zenoh::Config> get_z_config(const ConfigurableEntity & entity);

///=============================================================================
/// Get the number of times rmw_init should try to connect to a zenoh router
/// based on the environment variable ZENOH_ROUTER_CHECK_ATTEMPTS.
/// @details The behavior is as follows:
///   - If not set or 0, the max value is returned.
///   - If less than 0, std::nullopt is returned.
///   - Else value of environment variable is returned.
/// @return The number of times to try connecting to a zenoh router and
///   std::nullopt if establishing a connection to a router is not required.
std::optional<uint64_t> zenoh_router_check_attempts();
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__ZENOH_CONFIG_HPP_
