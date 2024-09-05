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

#include <zenoh.h>

#include <cstdlib>
#include <cstring>
#include <string>

#include "rmw/types.h"

#include "logging_macros.hpp"

#include "attachment_helpers.hpp"

namespace rmw_zenoh_cpp
{

bool create_attachment_iter(z_owned_bytes_t * kv_pair, void * context)
{
  attachement_context_t * ctx = reinterpret_cast<attachement_context_t *>(context);
  z_owned_bytes_t k, v;

  if (ctx->idx == 0) {
    z_bytes_serialize_from_str(&k, "sequence_number");
    z_bytes_serialize_from_int64(&v, ctx->data->sequence_number);
  } else if (ctx->idx == 1) {
    z_bytes_serialize_from_str(&k, "source_timestamp");
    z_bytes_serialize_from_int64(&v, ctx->data->source_timestamp);
  } else if (ctx->idx == 2) {
    z_bytes_serialize_from_str(&k, "source_gid");
    z_bytes_serialize_from_buf(
      &v, ctx->data->source_gid,
      RMW_GID_STORAGE_SIZE);
  } else {
    return false;
  }

  z_bytes_from_pair(kv_pair, z_move(k), z_move(v));
  ctx->idx += 1;
  return true;
}

z_result_t attachement_data_t::serialize_to_zbytes(z_owned_bytes_t * attachment)
{
  attachement_context_t context = attachement_context_t(this);
  return z_bytes_from_iter(
    attachment, create_attachment_iter,
    reinterpret_cast<void *>(&context));
}

bool get_attachment(
  const z_loaned_bytes_t * const attachment,
  const std::string & key, z_owned_bytes_t * val)
{
  if (z_bytes_is_empty(attachment)) {
    return false;
  }

  z_bytes_iterator_t iter = z_bytes_get_iterator(attachment);
  z_owned_bytes_t pair, key_;
  bool found = false;

  while (z_bytes_iterator_next(&iter, &pair)) {
    z_bytes_deserialize_into_pair(z_loan(pair), &key_, val);
    z_owned_string_t key_string;
    z_bytes_deserialize_into_string(z_loan(key_), &key_string);

    const char * key_string_ptr = z_string_data(z_loan(key_string));
    size_t key_string_len = z_string_len(z_loan(key_string));
    if (key_string_len == key.length() && strncmp(key_string_ptr, key.c_str(), key.length()) == 0) {
      found = true;
    }

    z_drop(z_move(pair));
    z_drop(z_move(key_));
    z_drop(z_move(key_string));

    if (found) {
      break;
    }
  }

  if (!found) {
    return false;
  }

  if (z_bytes_is_empty(z_loan(*val))) {
    return false;
  }

  return true;
}

bool get_gid_from_attachment(
  const z_loaned_bytes_t * const attachment,
  uint8_t gid[RMW_GID_STORAGE_SIZE])
{
  if (z_bytes_is_empty(attachment)) {
    return false;
  }

  z_owned_bytes_t val;
  if (!get_attachment(attachment, "source_gid", &val)) {
    return false;
  }

  z_owned_slice_t slice;
  z_bytes_deserialize_into_slice(z_loan(val), &slice);
  if (z_slice_len(z_loan(slice)) != RMW_GID_STORAGE_SIZE) {
    RMW_ZENOH_LOG_ERROR_NAMED("rmw_zenoh_cpp", "GID length mismatched.")
    return false;
  }
  memcpy(gid, z_slice_data(z_loan(slice)), z_slice_len(z_loan(slice)));

  z_drop(z_move(val));
  z_drop(z_move(slice));

  return true;
}

int64_t get_int64_from_attachment(
  const z_loaned_bytes_t * const attachment,
  const std::string & name)
{
  // A valid request must have had an attachment
  if (z_bytes_is_empty(attachment)) {
    return -1;
  }

  // TODO(yuyuan): This key should be specific
  z_owned_bytes_t val;
  if (!get_attachment(attachment, name, &val)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Failed to deserialize int64 from the attachment.")
    return false;
  }

  int64_t num;
  if (z_bytes_deserialize_into_int64(z_loan(val), &num)) {
    return -1;
  }

  if (num == 0) {
    // This is an error regardless; the client should never send this
    return -1;
  }

  z_drop(z_move(val));

  return num;
}

}  // namespace rmw_zenoh_cpp
