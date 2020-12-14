// Copyright 2020 ADLINK, Inc.
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

#ifndef RMW_ZENOH_COMMON_CPP__RMW_INIT_HPP_
#define RMW_ZENOH_COMMON_CPP__RMW_INIT_HPP_

#include "rmw/init.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/init_options.h"

#include "rmw_zenoh_common_cpp/visibility_control.h"

namespace rmw_zenoh_common_cpp
{

RMW_ZENOH_COMMON_CPP_PUBLIC
rmw_ret_t
rmw_init_options_init(
  const char * identifier, rmw_init_options_t * init_options, rcutils_allocator_t allocator);

RMW_ZENOH_COMMON_CPP_PUBLIC
rmw_ret_t
rmw_init_options_copy(
  const char * identifier, const rmw_init_options_t * src, rmw_init_options_t * dst);

RMW_ZENOH_COMMON_CPP_PUBLIC
rmw_ret_t
rmw_init_options_fini(const char * identifier, rmw_init_options_t * init_options);

}  // namespace rmw_zenoh_common_cpp

#endif  // RMW_ZENOH_COMMON_CPP__RMW_INIT_HPP_
