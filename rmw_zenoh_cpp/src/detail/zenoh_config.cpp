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

#include "zenoh_config.hpp"

#include <rcutils/env.h>

#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

#include "logging_macros.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rmw/impl/cpp/macros.hpp>

///=============================================================================
namespace rmw_zenoh_cpp
{
namespace
{
/// Map the configurable entity to a pair of environment variable name that
/// stores the absolute path to the Zenoh config and the default config filename.
/// Note: The default config file should be located within rmw_zenoh_cpp/config/.
static const std::unordered_map<ConfigurableEntity,
  std::pair<const char *, const char *>> envar_map = {
  {ConfigurableEntity::Session,
    {"ZENOH_SESSION_CONFIG_URI", "DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5"}},
  {ConfigurableEntity::Router, {"ZENOH_ROUTER_CONFIG_URI", "DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5"}}
};

static const char * router_check_attempts_envar = "ZENOH_ROUTER_CHECK_ATTEMPTS";

rmw_ret_t _get_z_config(
  const char * envar_name,
  const char * default_uri,
  z_owned_config_t * config)
{
  const char * configured_uri;
  const char * envar_uri;
  // Get the path to the zenoh configuration file from the environment variable.
  if (NULL != rcutils_get_env(envar_name, &envar_uri)) {
    // NULL is returned if everything is ok.
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Envar %s cannot be read.", envar_name);
    return RMW_RET_ERROR;
  }

  // If the environment variable is set, try to read the configuration from the file,
  // if the environment variable is not set use internal configuration
  configured_uri = envar_uri[0] != '\0' ? envar_uri : default_uri;

  // Try to read the configuration
  if (zc_config_from_file(config, configured_uri) != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Invalid configuration file %s", configured_uri);
    return RMW_RET_ERROR;
  }
  RMW_ZENOH_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "configured using configuration file %s", configured_uri);
  return RMW_RET_OK;
}
}  // namespace

///=============================================================================
rmw_ret_t get_z_config(const ConfigurableEntity & entity, z_owned_config_t * config)
{
  auto envar_map_it = envar_map.find(entity);
  if (envar_map_it == envar_map.end()) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "get_z_config called with invalid ConfigurableEntity.");
    return RMW_RET_ERROR;
  }
  // Get the absolute path to the default configuration file.
  static const std::string path_to_config_folder =
    ament_index_cpp::get_package_share_directory("rmw_zenoh_cpp") + "/config/";
  const std::string default_config_path = path_to_config_folder + envar_map_it->second.second;

  return _get_z_config(envar_map_it->second.first, default_config_path.c_str(), config);
}

///=============================================================================
std::optional<uint64_t> zenoh_router_check_attempts()
{
  const char * envar_value;
  // The default value is to check indefinitely.
  uint64_t default_value = std::numeric_limits<uint64_t>::max();

  if (NULL != rcutils_get_env(router_check_attempts_envar, &envar_value)) {
    // NULL is returned if everything is ok.
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Envar %s cannot be read. Report this bug.",
      router_check_attempts_envar);
    return default_value;
  }
  // If the environment variable contains a value, handle it accordingly.
  if (envar_value[0] != '\0') {
    const auto read_value = std::strtol(envar_value, nullptr, 10);
    if (read_value > 0) {
      return read_value;
    } else if (read_value < 0) {
      // If less than 0, we skip the check.
      return std::nullopt;
    }
    // If the value is 0, check indefinitely.
    return default_value;
  }

  // If unset, check indefinitely.
  return default_value;
}
}  // namespace rmw_zenoh_cpp
