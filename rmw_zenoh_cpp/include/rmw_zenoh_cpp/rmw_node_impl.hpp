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

#ifndef RMW_ZENOH_CPP__RMW_NODE_IMPL_HPP_
#define RMW_ZENOH_CPP__RMW_NODE_IMPL_HPP_

#include "rmw/rmw.h"

extern "C"
{
  #ifdef USE_ZENOH_PICO
  #include "zenoh.h"
#else
  #include "zenoh/zenoh.h"
#endif
}

struct rmw_node_impl_t
{
  rmw_guard_condition_t * graph_guard_condition_;
};

#endif  // RMW_ZENOH_CPP__RMW_NODE_IMPL_HPP_
