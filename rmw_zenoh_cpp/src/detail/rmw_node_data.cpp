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

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "logging_macros.hpp"

#include "rcpputils/scope_exit.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
std::shared_ptr<NodeData> NodeData::make(
  const rmw_node_t * const node,
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
      node,
      id,
      std::move(entity),
      std::move(token)
    });
}

///=============================================================================
NodeData::NodeData(
  const rmw_node_t * const node,
  std::size_t id,
  std::shared_ptr<liveliness::Entity> entity,
  zc_owned_liveliness_token_t token)
: node_(node),
  id_(std::move(id)),
  entity_(std::move(entity)),
  token_(std::move(token)),
  is_shutdown_(false),
  pubs_({})
{
  // Do nothing.
}

///=============================================================================
NodeData::~NodeData()
{
  const rmw_ret_t ret = this->shutdown();
  if (ret != RMW_RET_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Error destructing node /%s.",
      entity_->node_name().c_str()
    );
  }
}

///=============================================================================
std::size_t NodeData::id() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return id_;
}

///=============================================================================
bool NodeData::create_pub_data(
  const rmw_publisher_t * const publisher,
  z_session_t session,
  std::size_t id,
  const std::string & topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile)
{
  std::lock_guard<std::mutex> lock_guard(mutex_);
  if (is_shutdown_) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create PublisherData as the NodeData has been shutdown.");
    return false;
  }

  if (pubs_.count(publisher) > 0) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "PublisherData already exists.");
    return false;
  }

  auto pub_data = PublisherData::make(
    std::move(session),
    node_,
    entity_->node_info(),
    id_,
    std::move(id),
    std::move(topic_name),
    type_support,
    qos_profile);
  if (pub_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to make PublisherData.");
    return false;
  }

  auto insertion = pubs_.insert(std::make_pair(publisher, std::move(pub_data)));
  if (!insertion.second) {
    return false;
  }
  return true;
}

///=============================================================================
PublisherDataPtr NodeData::get_pub_data(const rmw_publisher_t * const publisher)
{
  std::lock_guard<std::mutex> lock_guard(mutex_);
  auto it = pubs_.find(publisher);
  if (it == pubs_.end()) {
    return nullptr;
  }

  return it->second;
}

///=============================================================================
void NodeData::delete_pub_data(const rmw_publisher_t * const publisher)
{
  std::lock_guard<std::mutex> lock_guard(mutex_);
  pubs_.erase(publisher);
}

///=============================================================================
bool NodeData::create_sub_data(
  const rmw_subscription_t * const subscription,
  z_session_t session,
  std::shared_ptr<GraphCache> graph_cache,
  std::size_t id,
  const std::string & topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile)
{
  std::lock_guard<std::mutex> lock_guard(mutex_);
  if (is_shutdown_) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create SubscriptionData as the NodeData has been shutdown.");
    return false;
  }

  if (subs_.count(subscription) > 0) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "SubscriptionData already exists.");
    return false;
  }

  auto sub_data = SubscriptionData::make(
    std::move(session),
    std::move(graph_cache),
    node_,
    entity_->node_info(),
    id_,
    std::move(id),
    std::move(topic_name),
    type_support,
    qos_profile);
  if (sub_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to make SubscriptionData.");
    return false;
  }

  auto insertion = subs_.insert(std::make_pair(subscription, std::move(sub_data)));
  if (!insertion.second) {
    return false;
  }
  return true;
}

///=============================================================================
SubscriptionDataPtr NodeData::get_sub_data(const rmw_subscription_t * const subscription)
{
  std::lock_guard<std::mutex> lock_guard(mutex_);
  auto it = subs_.find(subscription);
  if (it == subs_.end()) {
    return nullptr;
  }

  return it->second;
}

///=============================================================================
void NodeData::delete_sub_data(const rmw_subscription_t * const subscription)
{
  std::lock_guard<std::mutex> lock_guard(mutex_);
  subs_.erase(subscription);
}

///=============================================================================
rmw_ret_t NodeData::shutdown()
{
  std::lock_guard<std::mutex> lock(mutex_);
  rmw_ret_t ret = RMW_RET_OK;
  if (is_shutdown_) {
    return ret;
  }

  // Shutdown all the entities within this node.
  for (auto pub_it = pubs_.begin(); pub_it != pubs_.end(); ++pub_it) {
    ret = pub_it->second->shutdown();
    if (ret != RMW_RET_OK) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Unable to shutdown publisher %s within id %zu. rmw_ret_t code: %zu.",
        pub_it->second->topic_info().name_.c_str(),
        id_,
        ret
      );
    }
  }
  for (auto sub_it = subs_.begin(); sub_it != subs_.end(); ++sub_it) {
    ret = sub_it->second->shutdown();
    if (ret != RMW_RET_OK) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Unable to shutdown subscription %s within id %zu. rmw_ret_t code: %zu.",
        sub_it->second->topic_info().name_.c_str(),
        id_,
        ret
      );
    }
  }

  // Unregister this node from the ROS graph.
  zc_liveliness_undeclare_token(z_move(token_));

  is_shutdown_ = true;
  return ret;
}

///=============================================================================
// Check if the Node is shutdown.
bool NodeData::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}

}  // namespace rmw_zenoh_cpp
