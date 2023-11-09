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

#include "identifier.hpp"

namespace
{

/// Env var to set the path to the zenoh configuration file.
static constexpr const char * kZenohConfigFileEnvVar = "RMW_ZENOH_CONFIG_FILE";
/// The name of the default configuration file for the default zenoh session configuration.
static constexpr const char * const kDefaultZenohConfigFileName =
  "DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5";

}  // namespace

rmw_ret_t get_z_config(z_owned_config_t * config)
{
  const char * zenoh_config_path;
  // Get the path to the zenoh configuration file from the environment variable.
  if (NULL != rcutils_get_env(kZenohConfigFileEnvVar, &zenoh_config_path)) {
    // NULL is returned if everything is ok.
    RCUTILS_LOG_ERROR_NAMED(
      "ZenohConfiguration", "Env Var '%s' can't be read.", kZenohConfigFileEnvVar);
    return RMW_RET_ERROR;
  }

  // If the environment variable contains a path to a file, try to read the configuration from it.
  if (zenoh_config_path[0] != '\0') {
    // If the environment variable is set, try to read the configuration from the file.
    *config = zc_config_from_file(zenoh_config_path);
    RCUTILS_LOG_INFO_NAMED(
      "ZenohConfiguration",
      "Using zenoh configuration file pointed by '%s' envar: '%s'", kZenohConfigFileEnvVar,
      zenoh_config_path);
  } else {
    // If the environment variable is not set use internal configuration
    static const std::string path_to_config_folder =
      ament_index_cpp::get_package_share_directory(rmw_zenoh_identifier) + "/config/";
    const std::string default_zconfig_path = path_to_config_folder + kDefaultZenohConfigFileName;
    *config = zc_config_from_file(default_zconfig_path.c_str());
    RCUTILS_LOG_INFO_NAMED(
      "ZenohConfiguration",
      "Using default zenoh configuration file at '%s'", default_zconfig_path.c_str());
  }

  // Verify that the configuration is valid.
  if (!z_config_check(config)) {
    RCUTILS_LOG_ERROR_NAMED(
      "ZenohConfiguration",
      "Configuration is not valid. Please check the zenoh configuration file.");
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}
