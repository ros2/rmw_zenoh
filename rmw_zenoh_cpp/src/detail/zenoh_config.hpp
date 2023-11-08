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

#include <zenoh.h>

#include "rmw/ret_types.h"

/// Get the zenoh configuration for a session.
/// @details The behavior is as follows:
///   - If the environment variable `RMW_ZENOH_CONFIG_FILE` is set, it will
///     be used as the path to the zenoh configuration file.
///     - In case there is an error reading the file, the internal configuration will be used.
///   - If the environment variable `RMW_ZENOH_CONFIG_FILE` is not set, the
///     configuration will be read from the internal configuration.
///   - If internal configuration is not available, a zenoh default configuration is used.
/// @param config The zenoh configuration to be filled.
/// @returns `RMW_RET_OK` if the configuration was successfully loaded.
rmw_ret_t get_z_config(z_owned_config_t * config);

#endif  // DETAIL__ZENOH_CONFIG_HPP_
