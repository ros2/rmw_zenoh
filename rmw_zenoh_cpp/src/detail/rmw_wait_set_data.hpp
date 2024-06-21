// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef DETAIL__RMW_WAIT_SET_DATA_HPP_
#define DETAIL__RMW_WAIT_SET_DATA_HPP_

#include <condition_variable>
#include <mutex>

#include "rmw/rmw.h"

namespace rmw_zenoh_cpp
{

struct rmw_wait_set_data_t
{
  std::condition_variable condition_variable;
  std::mutex condition_mutex;

  bool triggered{false};

  rmw_context_t * context;
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__RMW_WAIT_SET_DATA_HPP_
