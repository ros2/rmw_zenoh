// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_ZENOH_DYNAMIC_CPP__GET_PARTICIPANT_HPP_
#define RMW_ZENOH_DYNAMIC_CPP__GET_PARTICIPANT_HPP_

#include "rmw_zenoh_common_cpp/zenoh-net-interface.h"
#include "rmw/rmw.h"
#include "rmw_zenoh_dynamic_cpp/visibility_control.h"

namespace rmw_zenoh_dynamic_cpp
{

RMW_ZENOH_DYNAMIC_CPP_PUBLIC
zn_session_t *
get_session(rmw_node_t * node);

}  // namespace rmw_zenoh_dynamic_cpp

#endif  // RMW_ZENOH_DYNAMIC_CPP__GET_PARTICIPANT_HPP_
