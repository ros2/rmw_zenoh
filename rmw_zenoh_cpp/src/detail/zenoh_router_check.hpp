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

#ifndef DETAIL__ZENOH_ROUTER_CHECK_HPP_
#define DETAIL__ZENOH_ROUTER_CHECK_HPP_

#include <zenoh.h>

#include "rmw/ret_types.h"

/// Check if a Zenoh router is connected to the session.
/// @param session Zenoh session to check.
/// @return RMW_RET_OK if a Zenoh router is connected to the session.
rmw_ret_t zenoh_router_check(z_session_t session);

#endif  // DETAIL__ZENOH_ROUTER_CHECK_HPP_
