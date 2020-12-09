// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_ZENOH_COMMON_CPP__RMW_CONTEXT_IMPL_HPP_
#define RMW_ZENOH_COMMON_CPP__RMW_CONTEXT_IMPL_HPP_

extern "C"
{
#include "rmw_zenoh_common_cpp/zenoh-net-interface.h"

extern zn_properties_t * configure_connection_mode(rmw_context_t * context);
extern void configure_session(zn_session_t * session);
}

struct rmw_context_impl_t
{
  zn_session_t * session;
  bool is_shutdown;
};

#endif  // RMW_ZENOH_COMMON_CPP__RMW_CONTEXT_IMPL_HPP_
