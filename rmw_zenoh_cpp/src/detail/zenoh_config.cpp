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
#include <rcutils/logging_macros.h>

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rmw/impl/cpp/macros.hpp>

///==============================================================================
namespace
{
/// The name of the default configuration file for the default zenoh session configuration.
static const std::unordered_map<ConfigurableEntity, const char *> default_config_filenames = {
  {ConfigurableEntity::Session, "DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5"},
  {ConfigurableEntity::Router, "DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5"}
};

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
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Envar %s cannot be read.", envar_name);
    return RMW_RET_ERROR;
  }
  // If the environment variable contains a path to a file, try to read the configuration from it.
  if (envar_uri[0] != '\0') {
    // If the environment variable is set, try to read the configuration from the file.
    *config = zc_config_from_file(envar_uri);
    configured_uri = envar_uri;
  } else {
    // If the environment variable is not set use internal configuration
    *config = zc_config_from_file(default_uri);
    configured_uri = default_uri;
  }
  // Verify that the configuration is valid.
  if (!z_config_check(config)) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Invalid configuration file %s", configured_uri);
    return RMW_RET_ERROR;
  }
  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "configured using configuration file %s", configured_uri);
  return RMW_RET_OK;
}
}  // namespace

///==============================================================================
rmw_ret_t get_z_config(const ConfigurableEntity & entity, z_owned_config_t * config)
{
  auto entity_envar_it = envar_map.find(entity);
  auto default_filename_it = default_config_filenames.find(entity);
  if (entity_envar_it == envar_map.end() ||
    default_filename_it == default_config_filenames.end() )
  {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "get_z_config called with invalid ConfigurableEntity.");
    return RMW_RET_ERROR;
  }
  // Get the absolute path to the default configuration file.
  static const std::string path_to_config_folder =
    ament_index_cpp::get_package_share_directory("rmw_zenoh_cpp") + "/config/";
  const std::string default_config_path = path_to_config_folder + default_filename_it->second;

  return _get_z_config(entity_envar_it->second, default_config_path.c_str(), config);
}
