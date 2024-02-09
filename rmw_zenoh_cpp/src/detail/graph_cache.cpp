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
#include <functional>
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
#include "rmw_data_types.hpp"

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
    [](const Entity & entity, GraphNode & graph_node, GraphCache & graph_cache) -> void
    {
      if (entity.type() == EntityType::Invalid ||
        entity.type() == EntityType::Node)
      {
        RCUTILS_LOG_WARN_NAMED(
          "rmw_zenoh_cpp",
          "add_topic_data() for invalid EntityType. Report this.");
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
      // For the sake of reusing data structures and lookup functions, we treat publishers and
      // clients are equivalent. Similarly, subscriptions and services are equivalent.
      const std::size_t pub_count = entity.type() == EntityType::Publisher ||
        entity.type() == EntityType::Client ? 1 : 0;
      const std::size_t sub_count = !pub_count;

      // Helper lambda to update TopicMap within the node the one for the entire graph.
      auto update_topic_map =
        [](GraphNode::TopicMap & topic_map,
          const liveliness::TopicInfo topic_info,
          const std::size_t pub_count,
          const std::size_t sub_count) -> void
        {
          TopicDataPtr graph_topic_data = std::make_shared<TopicData>(
            topic_info,
            TopicStats{pub_count, sub_count});
          std::string qos_str = liveliness::qos_to_keyexpr(topic_info.qos_);
          GraphNode::TopicQoSMap topic_qos_map = {
            {qos_str, graph_topic_data}};

          GraphNode::TopicMap::iterator topic_map_it = topic_map.find(topic_info.name_);
          if (topic_map_it == topic_map.end()) {
            // First time this topic name is discovered for the node so we insert a TopicTypeMap.
            GraphNode::TopicTypeMap topic_data_map = {
              {topic_info.type_, std::move(topic_qos_map)}
            };
            topic_map.insert(std::make_pair(topic_info.name_, std::move(topic_data_map)));
          } else {
            // The topic exists for the node so we check if the type also exists.
            GraphNode::TopicTypeMap::iterator topic_data_map_it = topic_map_it->second.find(
              topic_info.type_);
            if (topic_data_map_it == topic_map_it->second.end()) {
              // First time this topic type is added.
              topic_map_it->second.insert(
                std::make_pair(
                  topic_info.type_,
                  std::move(topic_qos_map)));
            } else {
              // The topic type already exists so we check if qos also exists.
              GraphNode::TopicQoSMap::iterator topic_qos_map_it = topic_data_map_it->second.find(
                qos_str);
              if (topic_qos_map_it == topic_data_map_it->second.end()) {
                // First time this qos is added.
                topic_data_map_it->second.insert(std::make_pair(qos_str, graph_topic_data));
              } else {
                // We have another instance of a pub/sub over the same topic,
                // type and qos so we increment the counters.
                TopicDataPtr & existing_graph_topic = topic_qos_map_it->second;
                existing_graph_topic->stats_.pub_count_ += pub_count;
                existing_graph_topic->stats_.sub_count_ += sub_count;
              }
            }
          }
        };

      // First update the topic map within the node.
      if (entity.type() == EntityType::Publisher) {
        update_topic_map(graph_node.pubs_, topic_info, pub_count, sub_count);
      } else if (entity.type() == EntityType::Subscription) {
        update_topic_map(graph_node.subs_, topic_info, pub_count, sub_count);
      } else if (entity.type() == EntityType::Service) {
        update_topic_map(graph_node.services_, topic_info, pub_count, sub_count);
      } else {
        update_topic_map(graph_node.clients_, topic_info, pub_count, sub_count);
      }

      // Then update the variables tracking topics across the graph.
      // TODO(Yadunund): Check for QoS events.
      if (entity.type() == EntityType::Publisher ||
        entity.type() == EntityType::Subscription)
      {
        update_topic_map(graph_cache.graph_topics_, topic_info, pub_count, sub_count);
      } else {
        update_topic_map(graph_cache.graph_services_, topic_info, pub_count, sub_count);
      }
    };

  // Helper lambda to convert an Entity into a GraphNode.
  // Note: this will update bookkeeping variables in GraphCache.
  auto make_graph_node =
    [&add_topic_data](const Entity & entity, GraphCache & graph_cache) -> std::shared_ptr<GraphNode>
    {
      auto graph_node = std::make_shared<GraphNode>();
      graph_node->id_ = entity.id();
      graph_node->ns_ = entity.node_namespace();
      graph_node->name_ = entity.node_name();
      graph_node->enclave_ = entity.node_enclave();

      if (!entity.topic_info().has_value()) {
        // Token was for a node.
        return graph_node;
      }
      // Add endpoint entries.
      add_topic_data(entity, *graph_node, graph_cache);

      return graph_node;
    };

  // Lock the graph mutex before accessing the graph.
  std::lock_guard<std::mutex> lock(graph_mutex_);

  // If the namespace did not exist, create it and add the node to the graph and return.
  NamespaceMap::iterator ns_it = graph_.find(entity.node_namespace());
  if (ns_it == graph_.end()) {
    NodeMap node_map = {
      {entity.node_name(), make_graph_node(entity, *this)}};
    graph_.emplace(std::make_pair(entity.node_namespace(), std::move(node_map)));
    total_nodes_in_graph_ += 1;
    return;
  }

  // Add the node to the namespace if it did not exist and return.
  // Case 1: First time a node with this name is added to the namespace.
  // Case 2: There are one or more nodes with the same name but the entity could
  // represent a node with the same name but a unique id which would make it a
  // new addition to the graph.
  std::pair<NodeMap::iterator, NodeMap::iterator> range = ns_it->second.equal_range(
    entity.node_name());
  NodeMap::iterator node_it = std::find_if(
    range.first, range.second,
    [&entity](const std::pair<std::string, GraphNodePtr> & node_it)
    {
      return entity.id() == node_it.second->id_;
    });
  if (node_it == range.second) {
    // Either the first time a node with this name is added or with an existing
    // name but unique id.
    NodeMap::iterator insertion_it =
      ns_it->second.insert(std::make_pair(entity.node_name(), make_graph_node(entity, *this)));
    total_nodes_in_graph_ += 1;
    if (insertion_it == ns_it->second.end()) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Unable to add a new node /%s with id %s an "
        "existing namespace %s in the graph. Report this bug.",
        entity.node_name().c_str(),
        entity.id().c_str(),
        entity.node_namespace().c_str());
    }
    return;
  }
  // Otherwise, the entity represents a node that already exists in the graph.
  // Update topic info if required below.

  // Handles additions to an existing node in the graph.
  if (entity.type() == EntityType::Node) {
    // Creating a new node above would have also updated the graph with any topic info.
    return;
  }

  if (!entity.topic_info().has_value()) {
    // Likely an error with parsing the token.
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Put token %s parsed without extracting topic_info. Report this bug.",
      keyexpr.c_str());
    return;
  }

  // Update the graph based on the entity.
  add_topic_data(entity, *(node_it->second), *this);
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

  // Helper lambda to update graph_topics_.
  auto update_topic_map =
    [](GraphNode::TopicMap & graph_endpoints,
      const liveliness::TopicInfo & topic_info,
      std::size_t pub_count,
      std::size_t sub_count) -> void
    {
      GraphNode::TopicMap::iterator cache_topic_it =
        graph_endpoints.find(topic_info.name_);
      if (cache_topic_it == graph_endpoints.end()) {
        // This should not happen.
        RCUTILS_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp", "topic_key %s not found in graph_endpoints. Report this.",
          topic_info.name_.c_str());
      } else {
        GraphNode::TopicTypeMap::iterator cache_topic_data_it =
          cache_topic_it->second.find(topic_info.type_);
        if (cache_topic_data_it != cache_topic_it->second.end()) {
          const std::string qos_str = liveliness::qos_to_keyexpr(topic_info.qos_);
          GraphNode::TopicQoSMap::iterator cache_topic_qos_it = cache_topic_data_it->second.find(
            qos_str);
          if (cache_topic_qos_it != cache_topic_data_it->second.end()) {
            // Decrement the relevant counters. If both counters are 0 remove from cache.
            cache_topic_qos_it->second->stats_.pub_count_ -= pub_count;
            cache_topic_qos_it->second->stats_.sub_count_ -= sub_count;
            if (cache_topic_qos_it->second->stats_.pub_count_ == 0 &&
              cache_topic_qos_it->second->stats_.sub_count_ == 0)
            {
              cache_topic_data_it->second.erase(qos_str);
            }
            // If the qos map is empty, erase it from the topic_data_map.
            if (cache_topic_data_it->second.empty()) {
              cache_topic_it->second.erase(cache_topic_data_it);
            }
          }
          // If the topic does not have any TopicData entries, erase the topic from the map.
          if (cache_topic_it->second.empty()) {
            graph_endpoints.erase(cache_topic_it);
          }
        }
      }
    };

  // Helper lambda to remove pub/subs to the GraphNode.
  // We capture by reference to update caches like graph_topics_ if update_cache is true.
  auto remove_topic_data =
    [&update_topic_map](const Entity & entity, GraphNode & graph_node,
      GraphCache & graph_cache) -> void
    {
      if (entity.type() == EntityType::Invalid ||
        entity.type() == EntityType::Node)
      {
        RCUTILS_LOG_WARN_NAMED(
          "rmw_zenoh_cpp",
          "remove_topic_data() for invalid EntityType. Report this.");
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
      // For the sake of reusing data structures and lookup functions, we treat publishers and
      // clients are equivalent. Similarly, subscriptions and services are equivalent.
      const std::size_t pub_count = entity.type() == EntityType::Publisher ||
        entity.type() == EntityType::Client ? 1 : 0;
      const std::size_t sub_count = !pub_count;

      // First update the topic map within the node.
      if (entity.type() == EntityType::Publisher) {
        update_topic_map(graph_node.pubs_, topic_info, pub_count, sub_count);
      } else if (entity.type() == EntityType::Subscription) {
        update_topic_map(graph_node.subs_, topic_info, pub_count, sub_count);
      } else if (entity.type() == EntityType::Service) {
        update_topic_map(graph_node.services_, topic_info, pub_count, sub_count);
      } else {
        update_topic_map(graph_node.clients_, topic_info, pub_count, sub_count);
      }

      // Then update the variables tracking topics across the graph.
      // TODO(Yadunund): Check for QoS events.
      if (entity.type() == EntityType::Publisher ||
        entity.type() == EntityType::Subscription)
      {
        update_topic_map(graph_cache.graph_topics_, topic_info, pub_count, sub_count);
      } else {
        update_topic_map(graph_cache.graph_services_, topic_info, pub_count, sub_count);
      }
    };

  // Lock the graph mutex before accessing the graph.
  std::lock_guard<std::mutex> lock(graph_mutex_);

  // If namespace does not exist, ignore the request.
  NamespaceMap::iterator ns_it = graph_.find(entity.node_namespace());
  if (ns_it == graph_.end()) {
    return;
  }

  // If the node does not exist, ignore the request.
  std::pair<NodeMap::iterator, NodeMap::iterator> range = ns_it->second.equal_range(
    entity.node_name());
  NodeMap::iterator node_it = std::find_if(
    range.first, range.second,
    [&entity](const std::pair<std::string, GraphNodePtr> & node_it)
    {
      // An operator== overload is defined above.
      return entity.id() == node_it.second->id_;
    });
  if (node_it == range.second) {
    // Node does not exist.
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "Received liveliness token to remove unknown node /%s from the graph. Ignoring...",
      entity.node_name().c_str()
    );
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
        "node have been removed. Removing all pub/subs first...",
        entity.node_name().c_str()
      );
      // We update the tracking variables to reduce the count of entities present in this node.
      auto remove_topic_map_from_cache =
        [&update_topic_map](const GraphNode::TopicMap & to_remove,
          GraphNode::TopicMap & from_cache) -> void
        {
          for (GraphNode::TopicMap::const_iterator topic_it = to_remove.begin();
            topic_it != to_remove.end(); ++topic_it)
          {
            for (GraphNode::TopicTypeMap::const_iterator topic_type_it = topic_it->second.begin();
              topic_type_it != topic_it->second.end(); ++topic_type_it)
            {
              for (GraphNode::TopicQoSMap::const_iterator topic_qos_it =
                topic_type_it->second.begin();
                topic_qos_it != topic_type_it->second.end(); ++topic_qos_it)
              {
                update_topic_map(
                  from_cache,
                  topic_qos_it->second->info_,
                  topic_qos_it->second->stats_.pub_count_,
                  topic_qos_it->second->stats_.sub_count_);
              }
            }
          }
        };
      remove_topic_map_from_cache(graph_node->pubs_, graph_topics_);
      remove_topic_map_from_cache(graph_node->subs_, graph_topics_);
      remove_topic_map_from_cache(graph_node->services_, graph_services_);
      remove_topic_map_from_cache(graph_node->clients_, graph_services_);
    }
    ns_it->second.erase(node_it);
    total_nodes_in_graph_ -= 1;
    if (ns_it->second.size() == 0) {
      graph_.erase(entity.node_namespace());
    }
    return;
  }

  if (!entity.topic_info().has_value()) {
    // Likely an error with parsing the token.
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Del token %s parsed without extracting TopicData. Report this bug.",
      keyexpr.c_str());
    return;
  }

  // Update the graph based on the entity.
  remove_topic_data(entity, *(node_it->second), *this);
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

  rcutils_ret_t rcutils_ret =
    rcutils_string_array_init(node_names, total_nodes_in_graph_, allocator);
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
    rcutils_string_array_init(node_namespaces, total_nodes_in_graph_, allocator);
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
      rcutils_string_array_init(enclaves, total_nodes_in_graph_, allocator);
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
  for (const std::pair<const std::string, GraphNode::TopicTypeMap> & item : entity_map) {
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
    for (const std::pair<const std::string, GraphNode::TopicQoSMap> & type : item.second) {
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
rmw_ret_t GraphCache::publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  // TODO(Yadunund): Replace this logic by returning a number that is tracked once
  // we support matched qos events.
  *subscription_count = 0;
  GraphNode::TopicMap::const_iterator topic_it = graph_topics_.find(publisher->topic_name);
  if (topic_it != graph_topics_.end()) {
    rmw_publisher_data_t * pub_data = static_cast<rmw_publisher_data_t *>(publisher->data);
    GraphNode::TopicTypeMap::const_iterator topic_data_it = topic_it->second.find(
      pub_data->type_support->get_name());
    if (topic_data_it != topic_it->second.end()) {
      for (const auto & [_, topic_data]  : topic_data_it->second) {
        // If a subscription exists with compatible QoS, update the subscription count.
        if (topic_data->stats_.sub_count_ > 0) {
          rmw_qos_compatibility_type_t is_compatible;
          rmw_ret_t ret = rmw_qos_profile_check_compatible(
            pub_data->adapted_qos_profile,
            topic_data->info_.qos_,
            &is_compatible,
            nullptr,
            0);
          if (ret == RMW_RET_OK && is_compatible == RMW_QOS_COMPATIBILITY_OK) {
            *subscription_count = *subscription_count + topic_data->stats_.sub_count_;
          }
        }
      }
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * publisher_count)
{
  // TODO(Yadunund): Replace this logic by returning a number that is tracked once
  // we support matched qos events.
  *publisher_count = 0;
  GraphNode::TopicMap::const_iterator topic_it = graph_topics_.find(subscription->topic_name);
  if (topic_it != graph_topics_.end()) {
    rmw_subscription_data_t * sub_data = static_cast<rmw_subscription_data_t *>(subscription->data);
    GraphNode::TopicTypeMap::const_iterator topic_data_it = topic_it->second.find(
      sub_data->type_support->get_name());
    if (topic_data_it != topic_it->second.end()) {
      for (const auto & [_, topic_data]  : topic_data_it->second) {
        // If a subscription exists with compatible QoS, update the subscription count.
        if (topic_data->stats_.pub_count_ > 0) {
          rmw_qos_compatibility_type_t is_compatible;
          rmw_ret_t ret = rmw_qos_profile_check_compatible(
            sub_data->adapted_qos_profile,
            topic_data->info_.qos_,
            &is_compatible,
            nullptr,
            0);
          if (ret == RMW_RET_OK && is_compatible == RMW_QOS_COMPATIBILITY_OK) {
            *publisher_count = *publisher_count + topic_data->stats_.pub_count_;
          }
        }
      }
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::get_service_names_and_types(
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * service_names_and_types) const
{
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);

  std::lock_guard<std::mutex> lock(graph_mutex_);
  return fill_names_and_types(graph_services_, allocator, service_names_and_types);
}

///=============================================================================
rmw_ret_t GraphCache::count_publishers(
  const char * topic_name,
  size_t * count) const
{
  *count = 0;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (graph_topics_.count(topic_name) != 0) {
    // Iterate through all the types and increment count.
    for (const std::pair<const std::string,
      GraphNode::TopicQoSMap> & topic_data : graph_topics_.at(topic_name))
    {
      for (auto it = topic_data.second.begin(); it != topic_data.second.end(); ++it) {
        *count += it->second->stats_.pub_count_;
      }
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
    // Iterate through all the types and increment count.
    for (const std::pair<const std::string,
      GraphNode::TopicQoSMap> & topic_data : graph_topics_.at(topic_name))
    {
      for (auto it = topic_data.second.begin(); it != topic_data.second.end(); ++it) {
        *count += it->second->stats_.sub_count_;
      }
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::count_services(
  const char * service_name,
  size_t * count) const
{
  *count = 0;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (graph_services_.count(service_name) != 0) {
    // Iterate through all the types and increment count.
    for (const std::pair<const std::string,
      GraphNode::TopicQoSMap> & topic_data : graph_services_.at(service_name))
    {
      for (auto it = topic_data.second.begin(); it != topic_data.second.end(); ++it) {
        *count += it->second->stats_.sub_count_;
      }
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::count_clients(
  const char * service_name,
  size_t * count) const
{
  *count = 0;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (graph_services_.count(service_name) != 0) {
    // Iterate through all the types and increment count.
    for (const std::pair<const std::string,
      GraphNode::TopicQoSMap> & topic_data : graph_services_.at(service_name))
    {
      for (auto it = topic_data.second.begin(); it != topic_data.second.end(); ++it) {
        *count += it->second->stats_.pub_count_;
      }
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
  // Since NodeMap is a multimap, this will return the first node with the same
  // name that is found.
  NodeMap::const_iterator node_it = ns_it->second.find(node_name);
  if (node_it == ns_it->second.end()) {
    return RMW_RET_OK;
  }

  // TODO(Yadunund): Support service and client when ready.
  if (entity_type == EntityType::Publisher) {
    return fill_names_and_types(node_it->second->pubs_, allocator, names_and_types);
  } else if (entity_type == EntityType::Subscription) {
    return fill_names_and_types(node_it->second->subs_, allocator, names_and_types);
  } else if (entity_type == EntityType::Service) {
    return fill_names_and_types(node_it->second->services_, allocator, names_and_types);
  } else if (entity_type == EntityType::Client) {
    return fill_names_and_types(node_it->second->clients_, allocator, names_and_types);
  } else {
    return RMW_RET_OK;
  }

  return RMW_RET_OK;
}


///=============================================================================
rmw_ret_t GraphCache::get_entities_info_by_topic(
  liveliness::EntityType entity_type,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_demangle,
  rmw_topic_endpoint_info_array_t * endpoints_info) const
{
  static_cast<void>(no_demangle);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);

  if (entity_type != EntityType::Publisher && entity_type != EntityType::Subscription) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  std::lock_guard<std::mutex> lock(graph_mutex_);

  // Minor optimization to exit early if the topic does not exist in the graph.
  if (graph_topics_.find(topic_name) == graph_topics_.end()) {
    return RMW_RET_OK;
  }
  // TODO(Yadunund): Refactor graph_topics_ to map to a list of GraphNodePtr to
  // avoid this expensive iteration.
  std::size_t size = 0;
  std::vector<GraphNodePtr> nodes = {};
  for (NamespaceMap::const_iterator ns_it = graph_.begin(); ns_it != graph_.end(); ++ns_it) {
    for (NodeMap::const_iterator node_it = ns_it->second.begin(); node_it != ns_it->second.end();
      ++node_it)
    {
      const GraphNode::TopicMap & entity_map =
        entity_type == EntityType::Publisher ? node_it->second->pubs_ :
        node_it->second->subs_;
      GraphNode::TopicMap::const_iterator topic_it = entity_map.find(topic_name);
      if (topic_it != entity_map.end()) {
        nodes.push_back(node_it->second);
        size += topic_it->second.size();
      }
    }
  }

  rmw_ret_t ret = rmw_topic_endpoint_info_array_init_with_size(
    endpoints_info,
    nodes.size(),
    allocator);
  if (RMW_RET_OK != ret) {
    return ret;
  }

  auto cleanup_endpoints_info = rcpputils::make_scope_exit(
    [endpoints_info, allocator] {
      rmw_ret_t fail_ret = rmw_topic_endpoint_info_array_fini(
        endpoints_info, allocator);
      if (fail_ret != RMW_RET_OK) {
        RMW_SAFE_FWRITE_TO_STDERR("failed to cleanup endpoints info during error handling");
      }
    });

  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const GraphNode::TopicMap & entity_map =
      entity_type == EntityType::Publisher ? nodes[i]->pubs_ :
      nodes[i]->subs_;
    const GraphNode::TopicTypeMap & topic_data_map = entity_map.find(topic_name)->second;
    for (const auto & [topic_type, topic_qos_map] : topic_data_map) {
      for (const auto & [_, topic_data] : topic_qos_map) {
        rmw_topic_endpoint_info_t & endpoint_info = endpoints_info->info_array[i];
        endpoint_info = rmw_get_zero_initialized_topic_endpoint_info();

        ret = rmw_topic_endpoint_info_set_node_name(
          &endpoint_info,
          nodes[i]->name_.c_str(),
          allocator);
        if (RMW_RET_OK != ret) {
          return ret;
        }

        ret = rmw_topic_endpoint_info_set_node_namespace(
          &endpoint_info,
          nodes[i]->ns_.c_str(),
          allocator);
        if (RMW_RET_OK != ret) {
          return ret;
        }

        ret = rmw_topic_endpoint_info_set_topic_type(
          &endpoint_info,
          _demangle_if_ros_type(topic_type).c_str(),
          allocator);
        if (RMW_RET_OK != ret) {
          return ret;
        }

        ret = rmw_topic_endpoint_info_set_endpoint_type(
          &endpoint_info,
          entity_type ==
          EntityType::Publisher ? RMW_ENDPOINT_PUBLISHER : RMW_ENDPOINT_SUBSCRIPTION);
        if (RMW_RET_OK != ret) {
          return ret;
        }

        ret = rmw_topic_endpoint_info_set_qos_profile(
          &endpoint_info,
          &topic_data->info_.qos_
        );
        if (RMW_RET_OK != ret) {
          return ret;
        }

        // TODO(Yadunund): Set type_hash, gid.
      }
    }
  }

  cleanup_endpoints_info.cancel();
  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::service_server_is_available(
  const char * service_name,
  const char * service_type,
  bool * is_available)
{
  *is_available = false;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  GraphNode::TopicMap::iterator service_it = graph_services_.find(service_name);
  if (service_it != graph_services_.end()) {
    GraphNode::TopicTypeMap::iterator type_it = service_it->second.find(service_type);
    if (type_it != service_it->second.end()) {
      for (const auto & [_, topic_data] : type_it->second) {
        if (topic_data->stats_.sub_count_ > 0) {
          *is_available = true;
          return RMW_RET_OK;
        }
      }
    }
  }

  return RMW_RET_OK;
}
