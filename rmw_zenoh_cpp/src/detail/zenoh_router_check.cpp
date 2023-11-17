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

namespace
{

// Convert a Zenoh Id to a string
// Zenoh IDs are LSB-first 128bit unsigned and non-zero integers in hexadecimal lowercase.
// @param pid Zenoh Id to convert
std::string zid_to_str(const z_id_t & pid)
{
  std::stringstream ss;
  int len = 0;
  for (size_t i = 0; i < sizeof(pid.id); ++i) {
    if (pid.id[i]) {
      len = static_cast<int>(i) + 1;
    }
  }
  if (!len) {
    ss << "";
  } else {
    for (int i = len - 1; i >= 0; --i) {
      ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(pid.id[i]);
    }
  }
  return ss.str();
}

}  // namespace

rmw_ret_t zenoh_router_check(z_session_t session)
{
  // Initialize context for callback
  int context = 0;

  // Define callback
  auto callback = [](const struct z_id_t * id, void * ctx) {
      const std::string id_str = zid_to_str(*id);
      RCUTILS_LOG_INFO_NAMED(
        "ZenohRouterCheck",
        "A Zenoh router connected to the session with id '%s'", id_str.c_str());
      // Note: Callback is guaranteed to never be called
      // concurrently according to z_info_routers_zid docstring
      (*(static_cast<int *>(ctx)))++;
    };

  rmw_ret_t ret;
  z_owned_closure_zid_t router_callback = z_closure(callback, nullptr /* drop */, &context);
  if (z_info_routers_zid(session, z_move(router_callback))) {
    RCUTILS_LOG_ERROR_NAMED(
      "ZenohRouterCheck",
      "Failed to evaluate if Zenoh routers are connected to the session");
    ret = RMW_RET_ERROR;
  } else {
    if (context == 0) {
      RCUTILS_LOG_ERROR_NAMED(
        "ZenohRouterCheck",
        "No Zenoh router connected to the session");
      ret = RMW_RET_ERROR;
    } else {
      RCUTILS_LOG_INFO_NAMED(
        "ZenohRouterCheck",
        "There are %d Zenoh routers connected to the session", context);
      ret = RMW_RET_OK;
    }
  }

  return ret;
}
