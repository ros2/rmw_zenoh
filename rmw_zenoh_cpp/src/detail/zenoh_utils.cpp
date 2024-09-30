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

#include "zenoh_utils.hpp"

#include <chrono>
#include <cinttypes>

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"

namespace rmw_zenoh_cpp
{
///=============================================================================
z_owned_bytes_map_t
create_map_and_set_sequence_num(int64_t sequence_number, const uint8_t gid[RMW_GID_STORAGE_SIZE])
{
  z_owned_bytes_map_t map = z_bytes_map_new();
  if (!z_check(map)) {
    RMW_SET_ERROR_MSG("failed to allocate map for sequence number");
    return z_bytes_map_null();
  }
  auto free_attachment_map = rcpputils::make_scope_exit(
    [&map]() {
      z_bytes_map_drop(z_move(map));
    });

  // The largest possible int64_t number is INT64_MAX, i.e. 9223372036854775807.
  // That is 19 characters long, plus one for the trailing \0, means we need 20 bytes.
  char seq_id_str[20];
  if (rcutils_snprintf(seq_id_str, sizeof(seq_id_str), "%" PRId64, sequence_number) < 0) {
    RMW_SET_ERROR_MSG("failed to print sequence_number into buffer");
    return z_bytes_map_null();
  }
  z_bytes_map_insert_by_copy(&map, z_bytes_new("sequence_number"), z_bytes_new(seq_id_str));

  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  char source_ts_str[20];
  if (rcutils_snprintf(source_ts_str, sizeof(source_ts_str), "%" PRId64, now_ns.count()) < 0) {
    RMW_SET_ERROR_MSG("failed to print sequence_number into buffer");
    return z_bytes_map_null();
  }
  z_bytes_map_insert_by_copy(&map, z_bytes_new("source_timestamp"), z_bytes_new(source_ts_str));

  z_bytes_t gid_bytes;
  gid_bytes.len = RMW_GID_STORAGE_SIZE;
  gid_bytes.start = gid;

  z_bytes_map_insert_by_copy(&map, z_bytes_new("source_gid"), gid_bytes);

  free_attachment_map.cancel();

  return map;
}
}  // namespace rmw_zenoh_cpp
