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

#include "rmw/error_handling.h"
#include "rmw/sanity_checks.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

#include "graph_cache.hpp"

///=============================================================================
using Entity = liveliness::Entity;
using EntityType = liveliness::EntityType;

///=============================================================================
TopicStats::TopicStats(std::size_t pub_count, std::size_t sub_count)
: pub_count_(pub_count),
  sub_count_(sub_count)
{
  // Do nothing.
}

///=============================================================================
TopicData::TopicData(
  liveliness::TopicInfo info,
  TopicStats stats)
: info_(std::move(info)),
  stats_(std::move(stats))
{}

///=============================================================================
void GraphCache::parse_put(const std::string & keyexpr)
{
  std::optional<liveliness::Entity> valid_entity = liveliness::Entity::make(keyexpr);
  if (!valid_entity.has_value()) {
    // Error message has already been logged.
    return;
  }
  const liveliness::Entity & entity = *valid_entity;

  // Helper lambda to append pub/subs to the GraphNode.
  // We capture by reference to update graph_topics_
  auto add_topic_data =
    [this](const Entity & entity, GraphNode & graph_node) -> void
    {
      if (entity.type() != EntityType::Publisher && entity.type() != EntityType::Subscription) {
        return;
      }

      if (!entity.topic_info().has_value()) {
        // This should not happen as add_topic_data() is called after validating the existence
        // of topic_info.
        RCUTILS_LOG_WARN_NAMED(
          "rmw_zenoh_cpp",
          "add_topic_data() called without valid TopicInfo. Report this.");
        return;
      }

      const liveliness::TopicInfo topic_info = entity.topic_info().value();
      GraphNode::TopicMap & topic_map = entity.type() ==
        EntityType::Publisher ? graph_node.pubs_ : graph_node.subs_;
      const std::string entity_desc = entity.type() ==
        EntityType::Publisher ? "publisher" : "subscription";
      const std::size_t pub_count = entity.type() == EntityType::Publisher ? 1 : 0;
      const std::size_t sub_count = !pub_count;
      TopicDataPtr graph_topic_data = std::make_shared<TopicData>(
        topic_info,
        TopicStats{pub_count, sub_count});

      GraphNode::TopicDataMap topic_data_map = {
        {graph_topic_data->info_.type_, graph_topic_data}};
      std::pair<GraphNode::TopicMap::iterator, bool> insertion =
        topic_map.insert(std::make_pair(topic_info.name_, topic_data_map));
      if (!insertion.second) {
        // A topic with the same name already exists in the node so we append the type.
        std::pair<GraphNode::TopicDataMap::iterator, bool> type_insertion =
          insertion.first->second.insert(
          std::make_pair(
            graph_topic_data->info_.type_,
            graph_topic_data));
        if (!type_insertion.second) {
          // We have another instance of a pub/sub over the same topic and type so we increment
          // the counters.
          TopicDataPtr & existing_graph_topic = type_insertion.first->second;
          existing_graph_topic->stats_.pub_count_ += pub_count;
          existing_graph_topic->stats_.sub_count_ += sub_count;
        }
      }

      // Bookkeeping: Update graph_topics_ which keeps track of topics across all nodes in the graph
      GraphNode::TopicMap::iterator cache_topic_it = graph_topics_.find(topic_info.name_);
      if (cache_topic_it == graph_topics_.end()) {
        // First time this topic name is added to the graph.
        std::shared_ptr<TopicData> topic_data_ptr = std::make_shared<TopicData>(
          topic_info,
          TopicStats{pub_count, sub_count}
        );
        graph_topics_[topic_info.name_] = GraphNode::TopicDataMap{
          {topic_info.type_, topic_data_ptr}
        };
      } else {
        // If a TopicData entry for the same type exists in the topic map, update pub/sub counts
        // or else create an new TopicData.
        std::pair<GraphNode::TopicDataMap::iterator, bool> topic_data_insertion =
          cache_topic_it->second.insert(std::make_pair(topic_info.type_, nullptr));
        if (topic_data_insertion.second) {
          // A TopicData for the topic_type does not exist.
          topic_data_insertion.first->second = std::make_shared<TopicData>(
            topic_info,
            TopicStats{pub_count, sub_count});
        } else {
          // Update the existing counters.
          topic_data_insertion.first->second->stats_.pub_count_ += pub_count;
          topic_data_insertion.first->second->stats_.sub_count_ += sub_count;
        }
      }

      RCUTILS_LOG_INFO_NAMED(
        "rmw_zenoh_cpp",
        "Added %s on topic %s with type %s and qos %s to node /%s.",
        entity_desc.c_str(),
        topic_info.name_.c_str(),
        topic_info.type_.c_str(),
        topic_info.qos_.c_str(),
        graph_node.name_.c_str());
    };

  // Helper lambda to convert an Entity into a GraphNode.
  auto make_graph_node =
    [&](const Entity & entity) -> std::shared_ptr<GraphNode>
    {
      auto graph_node = std::make_shared<GraphNode>();
      graph_node->ns_ = entity.node_namespace();
      graph_node->name_ = entity.node_name();
      graph_node->enclave_ = entity.node_enclave();

      if (!entity.topic_info().has_value()) {
        // Token was for a node.
        return graph_node;
      }
      // Add pub/sub entries.
      add_topic_data(entity, *graph_node);

      return graph_node;
    };

  // Lock the graph mutex before accessing the graph.
  std::lock_guard<std::mutex> lock(graph_mutex_);

  // If the namespace did not exist, create it and add the node to the graph and return.
  NamespaceMap::iterator ns_it = graph_.find(entity.node_namespace());
  if (ns_it == graph_.end()) {
    NodeMap node_map = {
      {entity.node_name(), make_graph_node(entity)}};
    graph_.emplace(std::make_pair(entity.node_namespace(), std::move(node_map)));
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp", "Added node /%s to a new namespace %s in the graph.",
      entity.node_name().c_str(),
      entity.node_namespace().c_str());
    return;
  }

  // Add the node to the namespace if it did not exist and return.
  NodeMap::iterator node_it = ns_it->second.find(entity.node_name());
  if (node_it == ns_it->second.end()) {
    ns_it->second.insert(std::make_pair(entity.node_name(), make_graph_node(entity)));
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp", "Added node /%s to an existing namespace %s in the graph.",
      entity.node_name().c_str(),
      entity.node_namespace().c_str());
    return;
  }

  // Handles additions to an existing node in the graph.
  if (entity.type() == EntityType::Node) {
    // The NN entity is implicitly handled above where we add the node.
    // If control reaches here, then we received a duplicate entry for the same node.
    // This could happen when we get() all the liveliness tokens when the node spins up and
    // receive a MP token before an NN one.
    return;
  }

  if (!entity.topic_info().has_value()) {
    // Likely an error with parsing the token.
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp", "Put token %s parsed without extracting topic_info.",
      keyexpr.c_str());
    return;
  }

  // Update the graph based on the entity.
  add_topic_data(entity, *(node_it->second));
}

///=============================================================================
void GraphCache::parse_del(const std::string & keyexpr)
{
  std::optional<liveliness::Entity> valid_entity = liveliness::Entity::make(keyexpr);
  if (!valid_entity.has_value()) {
    // Error message has already been logged.
    return;
  }
  const liveliness::Entity & entity = *valid_entity;

  // Helper lambda to append pub/subs to the GraphNode.
  // We capture by reference to update caches like graph_topics_ if update_cache is true.
  auto remove_topic_data =
    [&](const Entity & entity, GraphNode & graph_node,
      bool update_cache = false) -> void
    {
      if (entity.type() != EntityType::Publisher && entity.type() != EntityType::Subscription) {
        return;
      }

      if (!entity.topic_info().has_value()) {
        // This should not happen as add_topic_data() is called after validating the existence
        // of topic_info.
        RCUTILS_LOG_WARN_NAMED(
          "rmw_zenoh_cpp",
          "remove_topic_data() called without valid TopicInfo. Report this.");
        return;
      }

      const liveliness::TopicInfo topic_info = entity.topic_info().value();
      GraphNode::TopicMap & topic_map = entity.type() ==
        EntityType::Publisher ? graph_node.pubs_ : graph_node.subs_;
      const std::string entity_desc = entity.type() ==
        EntityType::Publisher ? "publisher" : "subscription";
      const std::size_t pub_count = entity.type() == EntityType::Publisher ? 1 : 0;
      const std::size_t sub_count = !pub_count;

      GraphNode::TopicMap::iterator topic_it = topic_map.find(topic_info.name_);
      if (topic_it == topic_map.end()) {
        // Pub/sub not found.
        return;
      }

      GraphNode::TopicDataMap & topic_data_map = topic_it->second;
      // Search the unordered_set for the TopicData for this topic.
      GraphNode::TopicDataMap::iterator topic_data_it =
        topic_data_map.find(topic_info.type_);
      if (topic_data_it == topic_data_map.end()) {
        // Something is wrong.
        RCUTILS_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp", "TopicData not found for topic %s. Report this.",
          topic_info.name_.c_str());
        return;
      }

      // Decrement the relevant counters. If both counters are 0 remove from graph_node.
      TopicDataPtr & existing_topic_data = topic_data_it->second;
      existing_topic_data->stats_.pub_count_ -= pub_count;
      existing_topic_data->stats_.sub_count_ -= sub_count;
      if (existing_topic_data->stats_.pub_count_ == 0 &&
        existing_topic_data->stats_.sub_count_ == 0)
      {
        topic_data_map.erase(topic_data_it);
      }
      // If the topic does not have any TopicData entries, erase the topic from the map.
      if (topic_data_map.empty()) {
        topic_map.erase(topic_info.name_);
      }

      // Bookkeeping: Update graph_topic_ which keeps track of topics across all nodes in the graph.
      if (update_cache) {
        GraphNode::TopicMap::iterator cache_topic_it =
          graph_topics_.find(topic_info.name_);
        if (cache_topic_it == graph_topics_.end()) {
          // This should not happen.
          RCUTILS_LOG_ERROR_NAMED(
            "rmw_zenoh_cpp", "topic_key %s not found in graph_topics_. Report this.",
            topic_info.name_.c_str());
        } else {
          GraphNode::TopicDataMap::iterator cache_topic_data_it =
            cache_topic_it->second.find(topic_info.type_);
          if (cache_topic_data_it != cache_topic_it->second.end()) {
            // Decrement the relevant counters. If both counters are 0 remove from cache.
            cache_topic_data_it->second->stats_.pub_count_ -= pub_count;
            cache_topic_data_it->second->stats_.sub_count_ -= sub_count;
            if (cache_topic_data_it->second->stats_.pub_count_ == 0 &&
              cache_topic_data_it->second->stats_.sub_count_ == 0)
            {
              cache_topic_it->second.erase(cache_topic_data_it);
            }
            // If the topic does not have any TopicData entries, erase the topic from the map.
            if (cache_topic_it->second.empty()) {
              graph_topics_.erase(cache_topic_it);
            }
          }
        }
      }

      RCUTILS_LOG_INFO_NAMED(
        "rmw_zenoh_cpp",
        "Removed %s on topic %s with type %s and qos %s to node /%s.",
        entity_desc.c_str(),
        topic_info.name_.c_str(),
        topic_info.type_.c_str(),
        topic_info.qos_.c_str(),
        graph_node.name_.c_str());
    };

  // Lock the graph mutex before accessing the graph.
  std::lock_guard<std::mutex> lock(graph_mutex_);

  // If namespace does not exist, ignore the request.
  NamespaceMap::iterator ns_it = graph_.find(entity.node_namespace());
  if (ns_it == graph_.end()) {
    return;
  }

  // If the node does not exist, ignore the request.
  NodeMap::iterator node_it = ns_it->second.find(entity.node_name());
  if (node_it == ns_it->second.end()) {
    return;
  }

  if (entity.type() == EntityType::Node) {
    // Node
    // The liveliness tokens to remove pub/subs should be received before the one to remove a node
    // given the reliability QoS for liveliness subs. However, if we find any pubs/subs present in
    // the node below, we should update the count in graph_topics_.
    const GraphNodePtr graph_node = node_it->second;
    if (!graph_node->pubs_.empty() || !graph_node->subs_.empty()) {
      RCUTILS_LOG_WARN_NAMED(
        "rmw_zenoh_cpp",
        "Received liveliness token to remove node /%s from the graph before all pub/subs for this "
        "node have been removed. Report this issue.",
        entity.node_name().c_str()
      );
      // TODO(Yadunund): Iterate through the nodes pubs_ and subs_ and decrement topic count in
      // graph_topics_.
    }
    ns_it->second.erase(entity.node_name());
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "Removed node /%s from the graph.",
      entity.node_name().c_str()
    );
    return;
  }

  if (!entity.topic_info().has_value()) {
    // Likely an error with parsing the token.
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp", "Del token %s parsed without extracting TopicData.",
      keyexpr.c_str());
    return;
  }

  // Update the graph based on the entity.
  remove_topic_data(entity, *(node_it->second), true);
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
  for (const std::pair<const std::string, NodeMap> & it : graph_) {
    nodes_number += it.second.size();
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
  for (NamespaceMap::const_iterator ns_it = graph_.begin(); ns_it != graph_.end(); ++ns_it) {
    const std::string & ns = ns_it->first;
    for (NodeMap::const_iterator node_it = ns_it->second.begin(); node_it != ns_it->second.end();
      ++node_it)
    {
      const GraphNodePtr node = node_it->second;
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

rmw_ret_t fill_names_and_types(
  const GraphNode::TopicMap & entity_map,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * names_and_types)
{
  const std::size_t entity_size = entity_map.size();
  rmw_ret_t ret = rmw_names_and_types_init(names_and_types, entity_size, allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  auto cleanup_names_and_types = rcpputils::make_scope_exit(
    [names_and_types] {
      rmw_ret_t fail_ret = rmw_names_and_types_fini(names_and_types);
      if (fail_ret != RMW_RET_OK) {
        RMW_SAFE_FWRITE_TO_STDERR("failed to cleanup names and types during error handling");
      }
    });
  // Fill topic names and types.
  std::size_t index = 0;
  for (const std::pair<const std::string, GraphNode::TopicDataMap> & item : entity_map) {
    names_and_types->names.data[index] = rcutils_strdup(item.first.c_str(), *allocator);
    if (!names_and_types->names.data[index]) {
      return RMW_RET_BAD_ALLOC;
    }
    {
      rcutils_ret_t rcutils_ret = rcutils_string_array_init(
        &names_and_types->types[index],
        item.second.size(),
        allocator);
      if (RCUTILS_RET_OK != rcutils_ret) {
        RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
        return RMW_RET_BAD_ALLOC;
      }
    }
    size_t type_index = 0;
    for (const std::pair<const std::string, TopicDataPtr> & type : item.second) {
      char * type_name = rcutils_strdup(_demangle_if_ros_type(type.first).c_str(), *allocator);
      if (!type_name) {
        RMW_SET_ERROR_MSG("failed to allocate memory for type name");
        return RMW_RET_BAD_ALLOC;
      }
      names_and_types->types[index].data[type_index] = type_name;
      ++type_index;
    }
    ++index;
  }

  cleanup_names_and_types.cancel();

  return RMW_RET_OK;
}

}  // namespace

///=============================================================================
rmw_ret_t GraphCache::get_topic_names_and_types(
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types) const
{
  static_cast<void>(no_demangle);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);

  std::lock_guard<std::mutex> lock(graph_mutex_);
  return fill_names_and_types(graph_topics_, allocator, topic_names_and_types);
}

///=============================================================================
rmw_ret_t GraphCache::count_publishers(
  const char * topic_name,
  size_t * count) const
{
  *count = 0;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (graph_topics_.count(topic_name) != 0) {
    for (const std::pair<const std::string, TopicDataPtr> & it : graph_topics_.at(topic_name)) {
      // Iterate through all the types and increment count.
      *count += it.second->stats_.pub_count_;
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::count_subscriptions(
  const char * topic_name,
  size_t * count) const
{
  *count = 0;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (graph_topics_.count(topic_name) != 0) {
    for (const std::pair<const std::string, TopicDataPtr> & it : graph_topics_.at(topic_name)) {
      // Iterate through all the types and increment count.
      *count += it.second->stats_.sub_count_;
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::get_entity_names_and_types_by_node(
  liveliness::EntityType entity_type,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * names_and_types) const
{
  static_cast<void>(no_demangle);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_NODE_NAME_VALID;
  rmw_ret_t ret = rmw_validate_node_name(node_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_NODE_NAME_VALID != validation_result) {
    const char * reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("node_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  validation_result = RMW_NAMESPACE_VALID;
  ret = rmw_validate_namespace(node_namespace, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_NAMESPACE_VALID != validation_result) {
    const char * reason = rmw_namespace_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("node_namespace argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  ret = rmw_names_and_types_check_zero(names_and_types);
  if (RMW_RET_OK != ret) {
    return ret;
  }

  std::lock_guard<std::mutex> lock(graph_mutex_);

  // Check if namespace exists.
  NamespaceMap::const_iterator ns_it = graph_.find(node_namespace);
  if (ns_it == graph_.end()) {
    return RMW_RET_OK;
  }

  // Check if node exists.
  NodeMap::const_iterator node_it = ns_it->second.find(node_name);
  if (node_it == ns_it->second.end()) {
    return RMW_RET_OK;
  }

  // TODO(Yadunund): Support service and client when ready.
  if (entity_type == EntityType::Publisher) {
    return fill_names_and_types(node_it->second->pubs_, allocator, names_and_types);
  } else if (entity_type == EntityType::Subscription) {
    return fill_names_and_types(node_it->second->subs_, allocator, names_and_types);
  } else {
    return RMW_RET_OK;
  }

  return RMW_RET_OK;
}
