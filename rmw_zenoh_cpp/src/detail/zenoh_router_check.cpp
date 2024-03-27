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

#include "zenoh_router_check.hpp"

#include <rcutils/env.h>
#include <rcutils/logging_macros.h>

#include <iomanip>
#include <sstream>
#include <string>

#include "liveliness_utils.hpp"

///=============================================================================
rmw_ret_t zenoh_router_check(z_session_t session)
{
  // Initialize context for callback
  int context = 0;

  // Define callback
  auto callback = [](const struct z_id_t * id, void * ctx) {
      const std::string id_str = liveliness::zid_to_str(*id);
      RCUTILS_LOG_INFO_NAMED(
        "rmw_zenoh_cpp",
        "Successfully connected to a Zenoh router with id %s.", id_str.c_str());
      // Note: Callback is guaranteed to never be called
      // concurrently according to z_info_routers_zid docstring
      (*(static_cast<int *>(ctx)))++;
    };

  rmw_ret_t ret = RMW_RET_OK;
  z_owned_closure_zid_t router_callback = z_closure(callback, nullptr /* drop */, &context);
  if (z_info_routers_zid(session, z_move(router_callback))) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Failed to evaluate if Zenoh routers are connected to the session.");
    ret = RMW_RET_ERROR;
  } else {
    if (context == 0) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Unable to connect to a Zenoh router. "
        "Have you started a router with `ros2 run rmw_zenoh_cpp rmw_zenohd`?");
      ret = RMW_RET_ERROR;
    }
  }

  return ret;
}
