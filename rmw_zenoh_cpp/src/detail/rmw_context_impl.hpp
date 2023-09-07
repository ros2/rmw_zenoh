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

#ifndef SRC__DETAIL__RMW_CONTEXT_IMPL_HPP
#define SRC__DETAIL__RMW_CONTEXT_IMPL_HPP

#include "rmw/rmw.h"

#include "zenoh.h"

struct rmw_context_impl_s
{
  // An owned session.
  z_owned_session_t session;

  /// Shutdown flag.
  bool is_shutdown;

  // Equivalent to rmw_dds_common::Context's guard condition
  /// Guard condition that should be triggered when the graph changes.
  rmw_guard_condition_t * graph_guard_condition;
};

#endif  // SRC__DETAIL__RMW_CONTEXT_IMPL_HPP
