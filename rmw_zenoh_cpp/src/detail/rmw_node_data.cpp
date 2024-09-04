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

#include "rmw_node_data.hpp"

#include <utility>

#include "logging_macros.hpp"

#include "rcpputils/scope_exit.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
NodeData::NodeData(
  std::size_t id,
  std::shared_ptr<liveliness::Entity> entity,
  zc_owned_liveliness_token_t token)
: id_(std::move(id)),
  entity_(std::move(entity)),
  token_(std::move(token))
{
  // Do nothing.
}

///=============================================================================
NodeData::~NodeData()
{
  zc_liveliness_undeclare_token(z_move(token_));
}

///=============================================================================
std::size_t NodeData::id() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return id_;
}
}  // namespace rmw_zenoh_cpp
