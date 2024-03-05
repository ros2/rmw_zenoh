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

#include "attachment_helpers.hpp"

bool get_gid_from_attachment(
  const z_attachment_t * const attachment, uint8_t gid[RMW_GID_STORAGE_SIZE])
{
  if (!z_check(*attachment)) {
    return false;
  }

  z_bytes_t index = z_attachment_get(*attachment, z_bytes_new("source_gid"));
  if (!z_check(index)) {
    return false;
  }

  if (index.len != RMW_GID_STORAGE_SIZE) {
    return false;
  }

  memcpy(gid, index.start, index.len);

  return true;
}

int64_t get_int64_from_attachment(
  const z_attachment_t * const attachment, const std::string & name)
{
  if (!z_check(*attachment)) {
    // A valid request must have had an attachment
    return -1;
  }

  z_bytes_t index = z_attachment_get(*attachment, z_bytes_new(name.c_str()));
  if (!z_check(index)) {
    return -1;
  }

  if (index.len < 1) {
    return -1;
  }

  if (index.len > 19) {
    // The number was larger than we expected
    return -1;
  }

  // The largest possible int64_t number is INT64_MAX, i.e. 9223372036854775807.
  // That is 19 characters long, plus one for the trailing \0, means we need 20 bytes.
  char int64_str[20];

  memcpy(int64_str, index.start, index.len);
  int64_str[index.len] = '\0';

  errno = 0;
  char * endptr;
  int64_t num = strtol(int64_str, &endptr, 10);
  if (num == 0) {
    // This is an error regardless; the client should never send this
    return -1;
  } else if (endptr == int64_str) {
    // No values were converted, this is an error
    return -1;
  } else if (*endptr != '\0') {
    // There was junk after the number
    return -1;
  } else if (errno != 0) {
    // Some other error occurred, which may include overflow or underflow
    return -1;
  }

  return num;
}
