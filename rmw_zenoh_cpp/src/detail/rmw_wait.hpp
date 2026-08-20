// Copyright 2026 ktyang512
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

#ifndef DETAIL__RMW_WAIT_HPP_
#define DETAIL__RMW_WAIT_HPP_

#include "rmw/rmw.h"

#include "rmw_wait_set_data.hpp"

namespace rmw_zenoh_cpp::detail
{

inline bool condition_attachment_scan_complete(const rmw_wait_set_data_t *)
{
  // An entity may have set triggered while the attachment scan was in progress. Do not overwrite
  // that notification here; rmw_wait() resets the flag under the mutex before starting the scan.
  return false;
}

}  // namespace rmw_zenoh_cpp::detail

#endif  // DETAIL__RMW_WAIT_HPP_
