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

#include <algorithm>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/find_and_replace.hpp"
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
/**
 * Generate a liveliness token for the particular entity.
 *
 * The liveliness tokens are in the form:
 *
 * @ros2_lv/<domainid>/<entity>/<namespace>/<nodename>
 *
 * Where:
 *  <domainid> - A number set by the user to "partition" graphs.  Roughly equivalent to the domain ID in DDS.
 *  <entity> - The type of entity.  This can be one of "NN" for a node, "MP" for a publisher, "MS" for a subscription, "SS" for a service server, or "SC" for a service client.
 *  <namespace> - The ROS namespace for this entity.  If the namespace is absolute, this function will add in an _ for later parsing reasons.
 *  <nodename> - The ROS node name for this entity.
 */
static std::string generate_base_token(
  const std::string & entity,
  size_t domain_id,
  const std::string & namespace_,
  const std::string & name)
{
  std::stringstream token_ss;
  token_ss << "@ros2_lv/" << domain_id << "/" << entity << namespace_;
  // An empty namespace from rcl will contain "/" but zenoh does not allow keys with "//".
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
  return token;
}

///=============================================================================
std::string GenerateToken::subscription(
  size_t domain_id,
  const std::string & node_namespace,
  const std::string & node_name,
  const std::string & topic,
  const std::string & type,
  const std::string & qos)
{
  std::string token = generate_base_token("MS", domain_id, node_namespace, node_name);
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
// An internal struct to bundle results of parsing a token.
struct TokenNode
{

  struct TokenTopicData
  {
    std::string name_;
    std::string type_;
    std::string qos_;

    TokenTopicData(
      std::string name,
      std::string type,
      std::string qos)
    : name_(std::move(name)),
      type_(std::move(type)),
      qos_(std::move(qos))
    {
      // Do nothing.
    }
  };

  std::string ns_;
  std::string name_;
  std::string enclave_;
  std::optional<TokenTopicData> topic_data_;

  TokenNode(
    std::string ns,
    std::string name,
    std::string enclave,
    std::optional<TokenTopicData> topic_data = std::nullopt)
  : ns_(std::move(ns)),
    name_(std::move(name)),
    enclave_(std::move(enclave)),
    topic_data_(std::move(topic_data))
  {
    // Do nothing.
  }
};

///=============================================================================
// Convert a liveliness token into a <entity, Node>
std::optional<std::pair<std::string, TokenNode>> _parse_token(const std::string & keyexpr)
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

  // Get the entity, ie NN, MP, MS, SS, SC.
  std::string & entity = parts[2];
  if (entity != "NN" && entity != "MP" && entity != "MS" && entity != "SS" && entity != "SC") {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received liveliness token with invalid entity [%s].", entity.c_str());
    return std::nullopt;
  }

  // TODO(Yadunund): Support enclaves.
  // Nodes with empty namespaces will contain a "_".
  TokenNode node{
    parts[3] == "_" ? "/" : "/" + parts[3],
    std::move(parts[4]),
    ""
  };

  if (entity != "NN") {
    if (parts.size() < 8) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Received invalid liveliness token");
      return std::nullopt;
    }
    if (entity == "MP" || entity == "MS" || entity == "SS" || entity == "SC") {
      TokenNode::TokenTopicData topic_data{
        "/" + std::move(parts[5]),
        std::move(parts[6]),
        std::move(parts[7])
      };
      node.topic_data_ = std::move(topic_data);
    } else {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Invalid entity [%s] in liveliness token", entity.c_str());
      return std::nullopt;
    }
  }

  return std::make_pair(std::move(entity), std::move(node));
}
}  // namespace


///=============================================================================
TopicStats::TopicStats(std::size_t pub_count, std::size_t sub_count)
: pub_count_(pub_count),
  sub_count_(sub_count)
{
  // Do nothing.
}


///=============================================================================
TopicData::TopicData(
  std::string type,
  std::string qos,
  TopicStats stats)
: type_(std::move(type)),
  qos_(std::move(qos)),
  stats_(std::move(stats))
{}

///=============================================================================
void GraphCache::parse_put(const std::string & keyexpr)
{
  auto valid_token = _parse_token(keyexpr);
  if (!valid_token.has_value()) {
    // Error message has already been logged.
    return;
  }
  const std::string & entity = valid_token->first;
  const auto & token_node = valid_token->second;

  // Helper lambda to append pub/subs to the GraphNode.
  // We capture by reference to update caches like graph_topics_ if update_cache is true.
  auto add_topic_data =
    [&](const TokenNode::TokenTopicData & topic_data, GraphNode & graph_node,
      const std::string & entity, bool update_cache = false) -> void
    {
      if (entity != "MP" && entity != "MS") {
        return;
      }

      auto & topic_map = entity == "MP" ? graph_node.pubs_ : graph_node.subs_;
      const std::string entity_desc = entity == "MP" ? "publisher" : "subscription";
      const std::size_t pub_count = entity == "MP" ? 1 : 0;
      const std::size_t sub_count = !pub_count;
      TopicDataPtr graph_topic_data = std::make_shared<TopicData>(
        topic_data.type_,
        topic_data.qos_,
        TopicStats{pub_count, sub_count});

      GraphNode::TopicDataSet topic_data_set = {graph_topic_data};
      auto insertion = topic_map.insert(std::make_pair(topic_data.name_, topic_data_set));
      if (!insertion.second) {
        // A topic with the same name already exists in the node so we append the type.
        auto type_insertion = insertion.first->second.insert(graph_topic_data);
        if (!type_insertion.second) {
          // We have another instance of a pub/sub over the same topic and type so we increment the counters.
          auto & existing_graph_topic = *(type_insertion.first);
          existing_graph_topic->stats_.pub_count_ += pub_count;
          existing_graph_topic->stats_.pub_count_ += sub_count;
        }
      }

      // Bookkeeping: We update graph_topic_ which keeps track of topics across all nodes in the graph.
      if (update_cache) {
        std::string topic_key = topic_data.name_ + "?" + topic_data.type_;
        auto cache_insertion = graph_topics_.insert(std::make_pair(std::move(topic_key), nullptr));
        if (cache_insertion.second) {
          // First entry for this topic name + type combo so we create a new TopicStats instance.
          cache_insertion.first->second = std::make_unique<TopicStats>(pub_count, sub_count);
        } else {
          // Else we update the existing counters.
          cache_insertion.first->second->pub_count_ += pub_count;
          cache_insertion.first->second->sub_count_ += sub_count;
        }
      }

      RCUTILS_LOG_INFO_NAMED(
        "rmw_zenoh_cpp",
        "Added %s on topic %s with type %s and qos %s to node /%s.",
        entity_desc.c_str(),
        topic_data.name_.c_str(),
        topic_data.type_.c_str(),
        topic_data.qos_.c_str(),
        graph_node.name_.c_str());
    };

  // Helper lambdas to convert a TokenNode into a basic GraphNode.
  auto make_graph_node =
    [&](const TokenNode & token_node, const std::string & entity) -> std::shared_ptr<GraphNode>
    {
      auto graph_node = std::make_shared<GraphNode>();
      graph_node->ns_ = token_node.ns_;
      graph_node->name_ = token_node.name_;
      graph_node->enclave_ = token_node.enclave_;

      if (!token_node.topic_data_.has_value()) {
        // Token was for a node.
        return graph_node;
      }
      // Add pub/sub entries.
      add_topic_data(token_node.topic_data_.value(), *graph_node, entity, true);

      return graph_node;
    };

  // Lock the graph mutex before accessing the graph.
  std::lock_guard<std::mutex> lock(graph_mutex_);

  // If the namespace did not exist, create it and add the node to the graph and return.
  auto ns_it = graph_.find(token_node.ns_);
  if (ns_it == graph_.end()) {
    std::unordered_map<std::string, GraphNodePtr> node_map = {
      {token_node.name_, make_graph_node(token_node, entity)}};
    graph_.insert(std::make_pair(token_node.ns_, std::move(node_map)));
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp", "Added node /%s to a new namespace %s in the graph.",
      token_node.name_.c_str(),
      token_node.ns_.c_str());
    return;
  }

  // Add the node to the namespace if it did not exist and return.
  auto node_it = ns_it->second.find(token_node.name_);
  if (node_it == ns_it->second.end()) {
    ns_it->second.insert(std::make_pair(token_node.name_, make_graph_node(token_node, entity)));
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp", "Added node /%s to an existing namespace %s in the graph.",
      token_node.name_.c_str(),
      token_node.ns_.c_str());
    return;
  }

  // Handles additions to an existing node in the graph.
  if (entity == "NN") {
    // The NN entity is implicitly handled above where we add the node.
    // If control reaches here, then we received a duplicate entry for the same node.
    // This could happen when we get() all the liveliness tokens when the node spins up and
    // receive a MP token before an NN one.
    return;
  }

  if (!token_node.topic_data_.has_value()) {
    // Likely an error with parsing the token.
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp", "Put token %s parsed without extracting TopicData.",
      keyexpr.c_str());
    return;
  }

  // Update the graph based on the entity.
  add_topic_data(token_node.topic_data_.value(), *(node_it->second), entity, true);
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
  const auto & token_node = valid_token->second;

  // Helper lambda to append pub/subs to the GraphNode.
  // We capture by reference to update caches like graph_topics_ if update_cache is true.
  auto remove_topic_data =
    [&](const TokenNode::TokenTopicData & topic_data, GraphNode & graph_node,
      const std::string & entity, bool update_cache = false) -> void
    {
      if (entity != "MP" && entity != "MS") {
        return;
      }

      auto & topic_map = entity == "MP" ? graph_node.pubs_ : graph_node.subs_;
      const std::string entity_desc = entity == "MP" ? "publisher" : "subscription";
      const std::size_t pub_count = entity == "MP" ? 1 : 0;
      const std::size_t sub_count = !pub_count;

      auto topic_it = topic_map.find(topic_data.name_);
      if (topic_it == topic_map.end()) {
        // Pub/sub not found.
        return;
      }

      auto & topic_data_set = topic_it->second;
      // Search the unordered_set for the TopicData for this topic.
      auto topic_data_it = std::find_if(
        topic_data_set.begin(), topic_data_set.end(),
        [&topic_data](const auto topic_data_ptr) {
          return topic_data.type_ == topic_data_ptr->type_;
        });
      if (topic_data_it == topic_data_set.end()) {
        // Something is wrong.
        RCUTILS_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp", "TopicData not found for topic %s. Report this.",
          topic_data.name_.c_str());
        return;
      }

      // Decrement the relevant counters. Check if both counters are 0 and if so remove from graph_node.
      auto & existing_topic_data = *topic_data_it;
      existing_topic_data->stats_.pub_count_ -= pub_count;
      existing_topic_data->stats_.sub_count_ -= sub_count;
      if (existing_topic_data->stats_.pub_count_ == 0 &&
        existing_topic_data->stats_.sub_count_ == 0)
      {
        topic_data_set.erase(topic_data_it);
      }
      // If the topic does not have any TopicData entries, erase the topic from the map.
      if (topic_data_set.empty()) {
        topic_map.erase(topic_data.name_);
      }

      // Bookkeeping: We update graph_topic_ which keeps track of topics across all nodes in the graph.
      if (update_cache) {
        std::string topic_key = topic_data.name_ + "?" + topic_data.type_;
        auto cache_topic_it = graph_topics_.find(topic_key);
        if (cache_topic_it == graph_topics_.end()) {
          // This should not happen.
          RCUTILS_LOG_ERROR_NAMED(
            "rmw_zenoh_cpp", "topic_key %s not found in graph_topics_. Report this.",
            topic_key.c_str());
        } else {
          // Decrement the relevant counters. Check if both counters are 0 and if so remove from cache.
          cache_topic_it->second->pub_count_ -= pub_count;
          cache_topic_it->second->sub_count_ -= sub_count;
          if (cache_topic_it->second->pub_count_ == 0 && cache_topic_it->second->sub_count_ == 0) {
            graph_topics_.erase(topic_key);
          }
        }
      }

      RCUTILS_LOG_INFO_NAMED(
        "rmw_zenoh_cpp",
        "Removed %s on topic %s with type %s and qos %s to node /%s.",
        entity_desc.c_str(),
        topic_data.name_.c_str(),
        topic_data.type_.c_str(),
        topic_data.qos_.c_str(),
        graph_node.name_.c_str());
    };

  // Lock the graph mutex before accessing the graph.
  std::lock_guard<std::mutex> lock(graph_mutex_);

  // If namespace does not exist, ignore the request.
  auto ns_it = graph_.find(token_node.ns_);
  if (ns_it == graph_.end()) {
    return;
  }

  // If the node does not exist, ignore the request.
  auto node_it = ns_it->second.find(token_node.name_);
  if (node_it == ns_it->second.end()) {
    return;
  }

  if (entity == "NN") {
    // Node
    // The liveliness tokens to remove pub/subs should be received before the one to remove a node
    // given the reliability QoS for liveliness subs. However, if we find any pubs/subs present in the node below,
    // we should update the count in graph_topics_.
    const auto graph_node = node_it->second;
    if (!graph_node->pubs_.empty() || !graph_node->subs_.empty()) {
      RCUTILS_LOG_WARN_NAMED(
        "rmw_zenoh_cpp",
        "Received liveliness token to remove node /%s from the graph before all pub/subs for this node have been removed. "
        "Report this issue.",
        token_node.name_.c_str()
      );
      // TODO(Yadunund): Iterate through the nodes pubs_ and subs_ and decrement topic count in graph_topics_.
    }
    ns_it->second.erase(token_node.name_);
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "Removed node /%s from the graph.",
      token_node.name_.c_str()
    );
    return;
  }

  if (!token_node.topic_data_.has_value()) {
    // Likely an error with parsing the token.
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp", "Del token %s parsed without extracting TopicData.",
      keyexpr.c_str());
    return;
  }

  // Update the graph based on the entity.
  remove_topic_data(token_node.topic_data_.value(), *(node_it->second), entity, true);
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
      node_names->data[j] = rcutils_strdup(node->name_.c_str(), *allocator);
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
          node->enclave_.c_str(), *allocator);
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

namespace
{
// Shamelessly copied from https://github.com/ros2/rmw_cyclonedds/blob/f7f67bdef82f59558366aa6ce94ef9af3c5ab569/rmw_cyclonedds_cpp/src/demangle.cpp#L67
std::string
_demangle_if_ros_type(const std::string & dds_type_string)
{
  if (dds_type_string[dds_type_string.size() - 1] != '_') {
    // not a ROS type
    return dds_type_string;
  }

  std::string substring = "dds_::";
  size_t substring_position = dds_type_string.find(substring);
  if (substring_position == std::string::npos) {
    // not a ROS type
    return dds_type_string;
  }

  std::string type_namespace = dds_type_string.substr(0, substring_position);
  type_namespace = rcpputils::find_and_replace(type_namespace, "::", "/");
  size_t start = substring_position + substring.size();
  std::string type_name = dds_type_string.substr(start, dds_type_string.length() - 1 - start);
  return type_namespace + type_name;
}

}  // namespace

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

  // Fill topic names and types.
  std::size_t j = 0;
  for (const auto & it : graph_topics_) {
    // Split based on "?".
    // TODO(Yadunund): Be more systematic about this.
    // TODO(clalancette): Rather than doing the splitting here, should we store
    // it in graph_topics_ already split?
    std::vector<std::string> parts = split_keyexpr(it.first, '?');
    if (parts.size() < 2) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Invalid topic_key %s", it.first.c_str());
      return RMW_RET_INVALID_ARGUMENT;
    }

    topic_names_and_types->names.data[j] = rcutils_strdup(parts[0].c_str(), *allocator);
    if (!topic_names_and_types->names.data[j]) {
      return RMW_RET_BAD_ALLOC;
    }

    // TODO(clalancette): This won't work if there are multiple types on the same topic
    rcutils_ret_t rcutils_ret = rcutils_string_array_init(
      &topic_names_and_types->types[j], 1, allocator);
    if (RCUTILS_RET_OK != rcutils_ret) {
      RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
      return RMW_RET_BAD_ALLOC;
    }

    topic_names_and_types->types[j].data[0] = rcutils_strdup(
      _demangle_if_ros_type(parts[1]).c_str(), *allocator);
    if (!topic_names_and_types->types[j].data[0]) {
      return RMW_RET_BAD_ALLOC;
    }

    ++j;
  }

  cleanup_names_and_types.cancel();

  return RMW_RET_OK;
}
