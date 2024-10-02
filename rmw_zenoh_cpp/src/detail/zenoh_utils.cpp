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

#include <chrono>

#include "zenoh_utils.hpp"
#include "attachment_helpers.hpp"
#include "logging_macros.hpp"

#include "rmw/types.h"

namespace rmw_zenoh_cpp
{
///=============================================================================
bool
create_map_and_set_sequence_num(
  z_owned_bytes_t * out_bytes,
  int64_t sequence_number,
  uint8_t gid[RMW_GID_STORAGE_SIZE])
{
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  int64_t source_timestamp = now_ns.count();

  rmw_zenoh_cpp::attachement_data_t data(sequence_number, source_timestamp, gid);
  if (data.serialize_to_zbytes(out_bytes)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Failed to serialize the attachment");
    return false;
  }

  return true;
}
}  // namespace rmw_zenoh_cpp
