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

#ifndef DETAIL__ATTACHMENT_HELPERS_HPP_
#define DETAIL__ATTACHMENT_HELPERS_HPP_

#include <zenoh.h>

#include <string>

#include "rmw/types.h"

namespace rmw_zenoh_cpp {

class attachement_data_t final {
public:
  int64_t sequence_number;
  int64_t source_timestamp;
  uint8_t source_gid[RMW_GID_STORAGE_SIZE];
  attachement_data_t(const int64_t _sequence_number,
                     const int64_t _source_timestamp,
                     const uint8_t _source_gid[RMW_GID_STORAGE_SIZE]) {
    sequence_number = _sequence_number;
    source_timestamp = _source_timestamp;
    memcpy(source_gid, _source_gid, RMW_GID_STORAGE_SIZE);
  }
  z_error_t serialize_to_zbytes(z_owned_bytes_t *);
};

class attachement_context_t final {
public:
  const attachement_data_t *data;
  int length = 3;
  int idx = 0;

  attachement_context_t(const attachement_data_t *_data) : data(_data) {}
};

bool get_attachment(const z_loaned_bytes_t *const attachment,
                    const std::string &key, z_owned_bytes_t *val);

bool get_gid_from_attachment(const z_loaned_bytes_t *const attachment,
                             uint8_t gid[RMW_GID_STORAGE_SIZE]);

int64_t get_int64_from_attachment(const z_loaned_bytes_t *const attachment,
                                  const std::string &name);
} // namespace rmw_zenoh_cpp

#endif // DETAIL__ATTACHMENT_HELPERS_HPP_
