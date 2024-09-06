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

#include <memory>
#include <string>
#include <utility>

#include "logging_macros.hpp"

#include "rcpputils/scope_exit.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
std::shared_ptr<NodeData> NodeData::make(
  std::size_t id,
  z_session_t session,
  std::size_t domain_id,
  const std::string & namespace_,
  const std::string & node_name,
  const std::string & enclave)
{
  // Create the entity.
  auto entity = rmw_zenoh_cpp::liveliness::Entity::make(
    z_info_zid(session),
    std::to_string(id),
    std::to_string(id),
    rmw_zenoh_cpp::liveliness::EntityType::Node,
    rmw_zenoh_cpp::liveliness::NodeInfo{
      domain_id,
      namespace_,
      node_name,
      enclave
    }
  );
  if (entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to make NodeData as node entity is invalid.");
    return nullptr;
  }

  // Create the liveliness token.
  zc_owned_liveliness_token_t token = zc_liveliness_declare_token(
    session,
    z_keyexpr(entity->liveliness_keyexpr().c_str()),
    NULL
  );
  auto free_token = rcpputils::make_scope_exit(
    [&token]() {
      z_drop(z_move(token));
    });
  if (!z_check(token)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the node.");
    return nullptr;
  }
  free_token.cancel();

  return std::shared_ptr<NodeData>(
    new NodeData{
      id,
      std::move(entity),
      std::move(token)
    });
}

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
