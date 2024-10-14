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
create_map_and_set_sequence_num(
  int64_t sequence_number,
  GIDCopier gid_copier,
  int64_t * source_timestamp)
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
  if (nullptr != source_timestamp) {
    *source_timestamp = now_ns.count();
  }
  char source_ts_str[20];
  if (rcutils_snprintf(source_ts_str, sizeof(source_ts_str), "%" PRId64, now_ns.count()) < 0) {
    RMW_SET_ERROR_MSG("failed to print source_timestamp into buffer");
    return z_bytes_map_null();
  }
  z_bytes_map_insert_by_copy(&map, z_bytes_new("source_timestamp"), z_bytes_new(source_ts_str));
  gid_copier(&map, "source_gid");

  free_attachment_map.cancel();

  return map;
}

///=============================================================================
ZenohQuery::ZenohQuery(const z_query_t * query)
{
  query_ = z_query_clone(query);
}

///=============================================================================
ZenohQuery::~ZenohQuery()
{
  z_drop(z_move(query_));
}

///=============================================================================
const z_query_t ZenohQuery::get_query() const
{
  return z_query_loan(&query_);
}

///=============================================================================
ZenohReply::ZenohReply(const z_owned_reply_t * reply)
{
  reply_ = *reply;
}

///=============================================================================
ZenohReply::~ZenohReply()
{
  z_reply_drop(z_move(reply_));
}

///=============================================================================
std::optional<z_sample_t> ZenohReply::get_sample() const
{
  if (z_reply_is_ok(&reply_)) {
    return z_reply_ok(&reply_);
  }

  return std::nullopt;
}
}  // namespace rmw_zenoh_cpp
