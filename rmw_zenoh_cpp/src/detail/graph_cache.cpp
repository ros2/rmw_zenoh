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

#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/scope_exit.hpp"

#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"

#include "rmw/sanity_checks.h"
#include "rmw/error_handling.h"

#include "graph_cache.hpp"

///=============================================================================
std::string GenerateToken::liveliness(size_t domain_id)
{
  std::string token = "@ros2_lv/" + std::to_string(domain_id) + "/**";
  return token;
}

///=============================================================================
static std::string generate_base_token(
  const std::string & entity,
  size_t domain_id,
  const std::string & namespace_,
  const std::string & name)
{
  std::stringstream token_ss;
  // TODO(Yadunund): Empty namespace will contain /. Fix non-empty namespace.
  token_ss << "@ros2_lv/" << domain_id << "/" << entity << namespace_ << name;
  return token_ss.str();
}

///=============================================================================
std::string GenerateToken::node(
  size_t domain_id,
  const std::string & namespace_,
  const std::string & name)
{
  return generate_base_token("NN", domain_id, namespace_, name);
}

///=============================================================================
std::string GenerateToken::publisher(
  size_t domain_id,
  const std::string & node_namespace,
  const std::string & node_name,
  const std::string & topic,
  const std::string & type,
  const std::string & qos)
{
  std::string token = generate_base_token("MP", domain_id, node_namespace, node_name);
  token += topic + "/" + type + "/" + qos;
  return token;
}

///=============================================================================
bool PublishToken::put(
  z_owned_session_t * session,
  const std::string & token)
{
  if (!z_session_check(session)) {
    RCUTILS_SET_ERROR_MSG("The zenoh session is invalid.");
    return false;
  }

  // TODO(Yadunund): z_keyexpr_new creates a copy so find a way to avoid it.
  z_owned_keyexpr_t keyexpr = z_keyexpr_new(token.c_str());
  auto drop_keyexpr = rcpputils::make_scope_exit(
    [&keyexpr]() {
      z_drop(z_move(keyexpr));
    });
  if (!z_keyexpr_check(&keyexpr)) {
    RCUTILS_SET_ERROR_MSG("invalid keyexpression generation for liveliness publication.");
    return false;
  }
  RCUTILS_LOG_WARN_NAMED("rmw_zenoh_cpp", "Sending PUT on %s", token.c_str());
  z_put_options_t options = z_put_options_default();
  options.encoding = z_encoding(Z_ENCODING_PREFIX_EMPTY, NULL);
  if (z_put(z_loan(*session), z_keyexpr(token.c_str()), nullptr, 0, &options) < 0) {
    RCUTILS_SET_ERROR_MSG("unable to publish liveliness for node creation");
    return false;
  }

  return true;
}

///=============================================================================
bool PublishToken::del(
  z_owned_session_t * session,
  const std::string & token)
{
  if (!z_session_check(session)) {
    RCUTILS_SET_ERROR_MSG("The zenoh session is invalid.");
    return false;
  }

  // TODO(Yadunund): z_keyexpr_new creates a copy so find a way to avoid it.
  z_owned_keyexpr_t keyexpr = z_keyexpr_new(token.c_str());
  auto drop_keyexpr = rcpputils::make_scope_exit(
    [&keyexpr]() {
      z_drop(z_move(keyexpr));
    });
  if (!z_keyexpr_check(&keyexpr)) {
    RCUTILS_SET_ERROR_MSG("invalid key-expression generation for liveliness publication.");
    return false;
  }
  RCUTILS_LOG_WARN_NAMED("rmw_zenoh_cpp", "Sending DELETE on %s", token.c_str());
  const z_delete_options_t options = z_delete_options_default();
  if (z_delete(z_loan(*session), z_loan(keyexpr), &options) < 0) {
    RCUTILS_SET_ERROR_MSG("failed to delete liveliness key");
    return false;
  }

  return true;
}

///=============================================================================
PublisherData::PublisherData(
  const char * topic, const char * node, const char * namespace_,
  const char * type, rcutils_allocator_t * allocator)
: allocator_(allocator)
{
  // TODO(clalancette): Check for error
  topic_name_ = rcutils_strdup(topic, *allocator);

  // TODO(clalancette): Check for error
  node_name_ = rcutils_strdup(node, *allocator);

  // TODO(clalancette): Check for error
  namespace_name_ = rcutils_strdup(namespace_, *allocator);

  // TODO(clalancette): Check for error
  type_name_ = rcutils_strdup(type, *allocator);
}

///=============================================================================
PublisherData::~PublisherData()
{
  allocator_->deallocate(topic_name_, allocator_->state);
  allocator_->deallocate(node_name_, allocator_->state);
  allocator_->deallocate(namespace_name_, allocator_->state);
  allocator_->deallocate(type_name_, allocator_->state);
}

///=============================================================================
SubscriptionData::SubscriptionData(
  const char * topic, const char * node, const char * namespace_,
  const char * type, rcutils_allocator_t * allocator)
: allocator_(allocator)
{
  // TODO(clalancette): Check for error
  topic_name_ = rcutils_strdup(topic, *allocator);

  // TODO(clalancette): Check for error
  node_name_ = rcutils_strdup(node, *allocator);

  // TODO(clalancette): Check for error
  namespace_name_ = rcutils_strdup(namespace_, *allocator);

  // TODO(clalancette): Check for error
  type_name_ = rcutils_strdup(type, *allocator);
}

///=============================================================================
SubscriptionData::~SubscriptionData()
{
  allocator_->deallocate(topic_name_, allocator_->state);
  allocator_->deallocate(node_name_, allocator_->state);
  allocator_->deallocate(namespace_name_, allocator_->state);
  allocator_->deallocate(type_name_, allocator_->state);
}

///=============================================================================
uint64_t
GraphCache::add_publisher(
  const char * topic, const char * node, const char * namespace_,
  const char * type, rcutils_allocator_t * allocator)
{
  std::lock_guard<std::mutex> lck(publishers_mutex_);
  uint64_t this_handle_id = publishers_handle_id_++;
  publishers_.emplace(
    std::make_pair(
      this_handle_id, std::make_unique<PublisherData>(topic, node, namespace_, type, allocator)));
  return this_handle_id;
}

///=============================================================================
void
GraphCache::remove_publisher(uint64_t handle)
{
  std::lock_guard<std::mutex> lck(publishers_mutex_);
  if (publishers_.count(handle) == 0) {
    return;
  }

  publishers_.erase(handle);
}

///=============================================================================
uint64_t
GraphCache::add_subscription(
  const char * topic, const char * node, const char * namespace_,
  const char * type, rcutils_allocator_t * allocator)
{
  std::lock_guard<std::mutex> lck(subscriptions_mutex_);
  uint64_t this_handle_id = subscriptions_handle_id_++;
  subscriptions_.emplace(
    std::make_pair(
      this_handle_id,
      std::make_unique<SubscriptionData>(topic, node, namespace_, type, allocator)));
  return this_handle_id;
}

///=============================================================================
void
GraphCache::remove_subscription(uint64_t handle)
{
  std::lock_guard<std::mutex> lck(subscriptions_mutex_);
  if (subscriptions_.count(handle) == 0) {
    return;
  }

  subscriptions_.erase(handle);
}

///=============================================================================
static std::vector<std::string> split_keyexpr(const std::string & keyexpr)
{
  std::vector<std::size_t> delim_idx = {};
  // Insert -1 for starting position to make the split easier when using substr.
  delim_idx.push_back(-1);
  std::size_t idx = 0;
  for (auto it = keyexpr.begin(); it != keyexpr.end(); ++it) {
    if (*it == '/') {
      delim_idx.push_back(idx);
    }
    ++idx;
  }
  std::vector<std::string> result = {};
  try {
    for (std::size_t i = 1; i < delim_idx.size(); ++i) {
      const auto & prev_idx = delim_idx.at(i - 1);
      const auto & idx = delim_idx.at(i);
      result.push_back(keyexpr.substr(prev_idx + 1, idx - prev_idx - 1));
    }
  } catch (const std::exception & e) {
    printf("%s\n", e.what());
    return {};
  }
  // Finally add the last substr.
  result.push_back(keyexpr.substr(delim_idx.back() + 1));
  return result;
}

///=============================================================================
void GraphCache::parse_put(const std::string & keyexpr)
{
  // TODO(Yadunund): Validate data.
  std::vector<std::string> parts = split_keyexpr(keyexpr);
  if (parts.size() < 3) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received invalid liveliness token");
    return;
  }
  // Get the entity, ie N, MP, MS, SS, SC.
  const std::string & entity = parts[2];
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (entity == "NN") {
    // Node
    RCUTILS_LOG_WARN_NAMED("rmw_zenoh_cpp", "Adding node %s to the graph.", parts.back().c_str());
    const bool has_namespace = entity.size() == 5 ? true : false;
    graph_[parts.back()] = YAML::Node();
    // TODO(Yadunund): Implement enclave support.
    graph_[parts.back()]["enclave"] = "";
    graph_[parts.back()]["namespace"] = has_namespace ? parts.at(4) : "/";
  } else if (entity == "MP") {
    // Publisher
  } else if (entity == "MS") {
    // Subscription
  } else if (entity == "SS") {
    // Service
  } else if (entity == "SC") {
    // Client
  } else {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received liveliness token with invalid entity type.");
    return;
  }
}

///=============================================================================
void GraphCache::parse_del(const std::string & keyexpr)
{
  // TODO(Yadunund): Validate data.
  std::vector<std::string> parts = split_keyexpr(keyexpr);
  if (parts.size() < 3) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received invalid liveliness token");
    return;
  }
  // Get the entity, ie N, MP, MS, SS, SC.
  const std::string & entity = parts[2];
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (entity == "NN") {
    // Node
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "Removing node %s from the graph.",
      parts.back().c_str()
    );
    graph_.remove(entity.back());
  } else if (entity == "MP") {
    // Publisher
  } else if (entity == "MS") {
    // Subscription
  } else if (entity == "SS") {
    // Service
  } else if (entity == "SC") {
    // Client
  } else {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received liveliness token with invalid entity type.");
    return;
  }
}

///=============================================================================
rmw_ret_t GraphCache::get_node_names(
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves,
  rcutils_allocator_t * allocator) const
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_names)) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_namespaces)) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (
    enclaves &&
    RMW_RET_OK != rmw_check_zero_rmw_string_array(enclaves))
  {
    return RMW_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);

  size_t nodes_number = graph_.size();

  rcutils_ret_t rcutils_ret =
    rcutils_string_array_init(node_names, nodes_number, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    return RMW_RET_BAD_ALLOC;
  }
  auto free_node_names = rcpputils::make_scope_exit(
    [node_names]() {
      rcutils_ret_t ret = rcutils_string_array_fini(node_names);
      if (ret != RCUTILS_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
      }
    });

  rcutils_ret =
    rcutils_string_array_init(node_namespaces, nodes_number, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    return RMW_RET_BAD_ALLOC;
  }
  auto free_node_namespaces = rcpputils::make_scope_exit(
    [node_namespaces]() {
      rcutils_ret_t ret = rcutils_string_array_fini(node_namespaces);
      if (ret != RCUTILS_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
      }
    });

  auto free_enclaves_lambda = [enclaves]() -> void {
      rcutils_ret_t ret = rcutils_string_array_fini(enclaves);
      if (ret != RCUTILS_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
      }
    };

  std::shared_ptr<rcpputils::scope_exit<decltype(free_enclaves_lambda)>> free_enclaves{nullptr};
  if (enclaves) {
    rcutils_ret =
      rcutils_string_array_init(enclaves, nodes_number, allocator);
    if (RCUTILS_RET_OK != rcutils_ret) {
      return RMW_RET_BAD_ALLOC;
    }
    free_enclaves =
      std::make_shared<rcpputils::scope_exit<decltype(free_enclaves_lambda)>>(
      std::move(free_enclaves_lambda));
  }

  // TODO(Yadunund): Remove this printout.
  const std::string & graph_str = YAML::Dump(graph_);
  RCUTILS_LOG_WARN_NAMED("rmw_zenoh_cpp", "[graph]\n%s\n", graph_str.c_str());
  // Fill node names, namespaces and enclaves.
  std::size_t j = 0;
  for (auto it = graph_.begin(); it != graph_.end(); ++it) {
    const auto & node_name = it->first.as<std::string>();
    const auto & yaml_node = it->second;
    node_names->data[j] = rcutils_strdup(node_name.c_str(), *allocator);
    if (!node_names->data[j]) {
      return RMW_RET_BAD_ALLOC;
    }
    node_namespaces->data[j] = rcutils_strdup(
      yaml_node["namespace"].as<std::string>().c_str(), *allocator);
    if (!node_namespaces->data[j]) {
      return RMW_RET_BAD_ALLOC;
    }
    if (enclaves) {
      enclaves->data[j] = rcutils_strdup(
        yaml_node["enclaves"].as<std::string>().c_str(), *allocator);
      if (!enclaves->data[j]) {
        return RMW_RET_BAD_ALLOC;
      }
    }
    ++j;
  }

  if (free_enclaves) {
    free_enclaves->cancel();
  }
  free_node_namespaces.cancel();
  free_node_names.cancel();

  return RMW_RET_OK;
}
