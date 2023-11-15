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
#include <optional>
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
  token_ss << "@ros2_lv/" << domain_id << "/" << entity << namespace_;
  // An empty namespace from rcl will contain "/"" but zenoh does not allow keys with "//".
  // Hence we add an "_" to denote an empty namespace such that splitting the key
  // will always result in 5 parts.
  if (namespace_ == "/") {
    token_ss << "_/";
  } else {
    token_ss << "/";
  }
  // Finally append node name.
  token_ss << name;
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
  printf("GenerateToken::Publisher: %s\n", token.c_str());
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
namespace
{
std::vector<std::string> split_keyexpr(
  const std::string & keyexpr,
  const char delim = '/')
{
  std::vector<std::size_t> delim_idx = {};
  // Insert -1 for starting position to make the split easier when using substr.
  delim_idx.push_back(-1);
  std::size_t idx = 0;
  for (auto it = keyexpr.begin(); it != keyexpr.end(); ++it) {
    if (*it == delim) {
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
// Convert a liveliness token into a <entity, Node>
std::optional<std::pair<std::string, GraphNode>> _parse_token(const std::string & keyexpr)
{
  std::vector<std::string> parts = split_keyexpr(keyexpr);
  // At minimum, a token will contain 5 parts (@ros2_lv, domain_id, entity, namespace, node_name).
  // Basic validation.
  if (parts.size() < 5) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received invalid liveliness token");
    return std::nullopt;
  }
  for (const std::string & p : parts) {
    if (p.empty()) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Received invalid liveliness token");
      return std::nullopt;
    }
  }

  // Get the entity, ie N, MP, MS, SS, SC.
  std::string & entity = parts[2];
  if (entity != "NN" &&
    entity != "MP" &&
    entity != "MS" &&
    entity != "SS" &&
    entity != "SC")
  {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Invalid entity [%s] in liveliness token", entity.c_str());
    return std::nullopt;
  }

  GraphNode node;
  // Nodes with empty namespaces will contain a "_".
  node.ns = parts.at(3) == "_" ? "/" : "/" + parts.at(3);
  node.name = std::move(parts[4]);

  if (entity != "NN") {
    if (parts.size() < 8) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Received invalid liveliness token");
      return std::nullopt;
    }
    GraphNode::PubSubData data;
    data.topic = "/" + std::move(parts[5]);
    data.type = std::move(parts[6]);
    data.qos = std::move(parts[7]);

    if (entity == "MP") {
      node.pubs.push_back(std::move(data));
    } else if (entity == "MS") {
      node.subs.push_back(std::move(data));
    } else {
      // TODO.
    }
  }

  return std::make_pair(std::move(entity), std::move(node));
}
} // namespace anonymous

///=============================================================================
void GraphCache::parse_put(const std::string & keyexpr)
{
  auto valid_token = _parse_token(keyexpr);
  if (!valid_token.has_value()) {
    // Error message has already been logged.
    return;
  }
  const std::string & entity = valid_token->first;
  auto node = std::make_shared<GraphNode>(std::move(valid_token->second));
  std::lock_guard<std::mutex> lock(graph_mutex_);

  if (entity == "NN") {
    // Node
    auto ns_it = graph_.find(node->ns);
    if (ns_it == graph_.end()) {
      // New namespace.
      std::string ns = node->ns;
      std::unordered_map<std::string, GraphNodePtr> map = {
        {node->name, node}
      };
      graph_.insert(std::make_pair(std::move(ns), std::move(map)));
    } else {
      auto insertion = ns_it->second.insert(std::make_pair(node->name, node));
      if (!insertion.second) {
        RCUTILS_LOG_WARN_NAMED(
          "rmw_zenoh_cpp", "Unable to add duplicate node /%s to the graph.",
          node->name.c_str());
      }
    }
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp", "Added node /%s to the graph.",
      node->name.c_str());
    return;

  } else if (entity == "MP") {
    // Publisher
    auto ns_it = graph_.find(node->ns);
    if (ns_it == graph_.end()) {
      // Potential edge case where a liveliness update for a node creation was missed.
      // So we add the node here.
      std::string ns = node->ns;
      std::unordered_map<std::string, GraphNodePtr> map = {
        {node->name, node}
      };
      graph_.insert(std::make_pair(std::move(ns), std::move(map)));
    } else {
      auto insertion = ns_it->second.insert(std::make_pair(node->name, node));
      if (!insertion.second && !node->pubs.empty()) {
        // Node already exists so just append the publisher.
        insertion.first->second->pubs.push_back(node->pubs[0]);
      }
    }
    // Bookkeeping
    // TODO(Yadunund): Be more systematic about generating the key.
    std::string topic_key = node->pubs.at(0).topic + "?" + node->pubs.at(0).type;
    auto insertion = graph_topics_.insert(std::make_pair(std::move(topic_key), 1));
    if (!insertion.second) {
      // Such a topic already exists so we just increment its count.
      ++insertion.first->second;
    }
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp", "Added publisher %s to node /%s in graph.",
      node->pubs.at(0).topic.c_str(),
      node->name.c_str());
    return;
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
  auto valid_token = _parse_token(keyexpr);
  if (!valid_token.has_value()) {
    // Error message has already been logged.
    return;
  }
  const std::string & entity = valid_token->first;
  auto node = std::make_shared<GraphNode>(std::move(valid_token->second));
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (entity == "NN") {
    // Node
    auto ns_it = graph_.find(node->ns);
    if (ns_it != graph_.end()) {
      ns_it->second.erase(node->name);
    }
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "Removed node /%s from the graph.",
      node->name.c_str()
    );
  } else if (entity == "MP") {
    // Publisher
    if (node->pubs.empty()) {
      // This should never happen but we make sure _parse_token() has no error.
      return;
    }
    auto ns_it = graph_.find(node->ns);
    if (ns_it != graph_.end()) {
      auto node_it = ns_it->second.find(node->name);
      if (node_it != ns_it->second.end()) {
        const auto found_node = node_it->second;
        // Here we iterate throught the list of publishers and remove the one
        // with matching name, type and qos.
        // TODO(Yadunund): This can be more optimal than O(n) with some caching.
        auto erase_it = found_node->pubs.begin();
        for (; erase_it != found_node->pubs.end(); ++erase_it) {
          const auto & pub = *erase_it;
          if (pub.topic == node->pubs.at(0).topic &&
            pub.type == node->pubs.at(0).type &&
            pub.qos == node->pubs.at(0).qos)
          {
            break;
          }
        }
        if (erase_it != found_node->pubs.end()) {
          found_node->pubs.erase(erase_it);
          // Bookkeeping
          // TODO(Yadunund): Be more systematic about generating the key.
          std::string topic_key = node->pubs.at(0).topic + "?" + node->pubs.at(0).type;
          auto topic_it = graph_topics_.find(topic_key);
          if (topic_it != graph_topics_.end()) {
            if (topic_it->second == 1) {
              // The last publisher was removed so we can delete this entry.
              graph_topics_.erase(topic_key);
            } else {
              // Else we just decrement the count.
              --topic_it->second;
            }
          }
          RCUTILS_LOG_WARN_NAMED(
            "rmw_zenoh_cpp",
            "Removed publisher %s from node /%s in the graph.",
            node->pubs.at(0).topic.c_str(),
            node->name.c_str()
          );
        }
      }
    }
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

  size_t nodes_number = 0;
  for (auto it = graph_.begin(); it != graph_.end(); ++it) {
    nodes_number += it->second.size();
  }
  // TODO(Yadunund): Delete.
  RCUTILS_LOG_WARN_NAMED(
    "rmw_zenoh_cpp",
    "nodes_number %ld", nodes_number);

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

  // Fill node names, namespaces and enclaves.
  std::size_t j = 0;
  for (auto ns_it = graph_.begin(); ns_it != graph_.end(); ++ns_it) {
    const std::string & ns = ns_it->first;
    for (auto node_it = ns_it->second.begin(); node_it != ns_it->second.end(); ++node_it) {
      const auto node = node_it->second;
      node_names->data[j] = rcutils_strdup(node->name.c_str(), *allocator);
      if (!node_names->data[j]) {
        return RMW_RET_BAD_ALLOC;
      }
      node_namespaces->data[j] = rcutils_strdup(
        ns.c_str(), *allocator);
      if (!node_namespaces->data[j]) {
        return RMW_RET_BAD_ALLOC;
      }
      if (enclaves) {
        enclaves->data[j] = rcutils_strdup(
          node->enclave.c_str(), *allocator);
        if (!enclaves->data[j]) {
          return RMW_RET_BAD_ALLOC;
        }
      }
      ++j;
    }
  }

  if (free_enclaves) {
    free_enclaves->cancel();
  }
  free_node_namespaces.cancel();
  free_node_names.cancel();

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::get_topic_names_and_types(
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  static_cast<void>(no_demangle);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);

  std::lock_guard<std::mutex> lock(graph_mutex_);
  const size_t topic_number = graph_topics_.size();
  // TODO(Yadunund): Delete.
  RCUTILS_LOG_WARN_NAMED(
    "rmw_zenoh_cpp",
    "topic_number %ld", topic_number);

  rmw_ret_t ret = rmw_names_and_types_init(topic_names_and_types, topic_number, allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  auto cleanup_names_and_types = rcpputils::make_scope_exit(
    [topic_names_and_types] {
      rmw_ret_t fail_ret = rmw_names_and_types_fini(topic_names_and_types);
      if (fail_ret != RMW_RET_OK) {
        RMW_SAFE_FWRITE_TO_STDERR("failed to cleanup names and types during error handling");
      }
    });

  // topic_names_and_types->names is an rcutils_string_array_t,
  // while topic_names_and_types->types is an rcutils_string_array_t *

  // rcutils_ret_t rcutils_ret =
  //   rcutils_string_array_init(topic_names_and_types->names, topic_number, allocator);
  // if (rcutils_ret != RCUTILS_RET_OK) {
  //   return RMW_RET_BAD_ALLOC;
  // }
  auto free_topic_names = rcpputils::make_scope_exit(
    [names = &topic_names_and_types->names]() {
      rcutils_ret_t ret = rcutils_string_array_fini(names);
      if (ret != RCUTILS_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
      }
    });

  rcutils_ret_t rcutils_ret =
    rcutils_string_array_init(topic_names_and_types->types, topic_number, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    return RMW_RET_BAD_ALLOC;
  }
  auto free_topic_types = rcpputils::make_scope_exit(
    [types = topic_names_and_types->types]() {
      rcutils_ret_t ret = rcutils_string_array_fini(types);
      if (ret != RCUTILS_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
      }
    });

  // Fill topic names and types.
  std::size_t j = 0;
  for (const auto & it : graph_topics_) {
    // Split based on "?".
    // TODO(Yadunund): Be more systematic about this.
    std::vector<std::string> parts = split_keyexpr(it.first, '?');
    if (parts.size() < 2) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Invalid topic_key %s", it.first.c_str());
      return RMW_RET_INVALID_ARGUMENT;
    }
    topic_names_and_types->names.data[j] = rcutils_strdup(parts.at(0).c_str(), *allocator);
    if (!topic_names_and_types->names.data[j]) {
      return RMW_RET_BAD_ALLOC;
    }
    topic_names_and_types->types->data[j] = rcutils_strdup(
      parts.at(1).c_str(), *allocator);
    if (!topic_names_and_types->types->data[j]) {
      return RMW_RET_BAD_ALLOC;
    }
    ++j;
  }

  cleanup_names_and_types.cancel();
  free_topic_names.cancel();
  free_topic_types.cancel();

  return RMW_RET_OK;
}
