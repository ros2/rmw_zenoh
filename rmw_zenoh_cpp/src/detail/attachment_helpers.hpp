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

bool get_gid_from_attachment(
  const z_attachment_t * const attachment, uint8_t gid[RMW_GID_STORAGE_SIZE]);

int64_t get_int64_from_attachment(
  const z_attachment_t * const attachment, const std::string & name);

#endif  // DETAIL__ATTACHMENT_HELPERS_HPP_
