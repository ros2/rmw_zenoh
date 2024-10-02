// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#ifndef DETAIL__ZENOH_UTILS_HPP_
#define DETAIL__ZENOH_UTILS_HPP_

#include <zenoh.h>

#include "rmw/types.h"

namespace rmw_zenoh_cpp
{
///=============================================================================
void
create_map_and_set_sequence_num(
  z_owned_bytes_t * out_bytes, int64_t sequence_number,
  uint8_t gid[RMW_GID_STORAGE_SIZE]);
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__ZENOH_UTILS_HPP_
