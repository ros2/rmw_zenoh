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

#ifndef DETAIL__GRAPH_CACHE_HPP_
#define DETAIL__GRAPH_CACHE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "liveliness_utils.hpp"

#include "rcutils/allocator.h"
#include "rcutils/types.h"

#include "rmw/rmw.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/names_and_types.h"


///=============================================================================
// TODO(Yadunund): Since we reuse pub_count_ and sub_count_ for pub/sub and
// service/client consider more general names for these fields.
struct TopicStats
{
  // The count of publishers or clients.
  std::size_t pub_count_;

  // The count of subscriptions or services.
  std::size_t sub_count_;

  // Constructor which initializes counters to 0.
  TopicStats(std::size_t pub_count, std::size_t sub_count);
};

///=============================================================================
struct TopicData
{
  liveliness::TopicInfo info_;
  TopicStats stats_;

  TopicData(
    liveliness::TopicInfo info,
    TopicStats stats);
};
using TopicDataPtr = std::shared_ptr<TopicData>;

///=============================================================================
// TODO(Yadunund): Expand to services and clients.
struct GraphNode
{
  std::string id_;
  std::string ns_;
  std::string name_;
  // TODO(Yadunund): Should enclave be the parent to the namespace key and not within a Node?
  std::string enclave_;

  // Map topic type to TopicData
  using TopicDataMap = std::unordered_map<std::string, TopicDataPtr>;
  // Map topic name to TopicDataMap
  using TopicMap = std::unordered_map<std::string, TopicDataMap>;

  // Entries for pub/sub.
  TopicMap pubs_ = {};
  TopicMap subs_ = {};

  // Entires for service/client.
  TopicMap clients_ = {};
  TopicMap services_ = {};

};
using GraphNodePtr = std::shared_ptr<GraphNode>;

///=============================================================================
class GraphCache final
{
public:
  // Parse a PUT message over a token's key-expression and update the graph.
  void parse_put(const std::string & keyexpr);
  // Parse a DELETE message over a token's key-expression and update the graph.
  void parse_del(const std::string & keyexpr);

  rmw_ret_t get_node_names(
    rcutils_string_array_t * node_names,
    rcutils_string_array_t * node_namespaces,
    rcutils_string_array_t * enclaves,
    rcutils_allocator_t * allocator) const;

  rmw_ret_t get_topic_names_and_types(
    rcutils_allocator_t * allocator,
    bool no_demangle,
    rmw_names_and_types_t * topic_names_and_types) const;

  rmw_ret_t publisher_count_matched_subscriptions(
    const rmw_publisher_t * publisher,
    size_t * subscription_count);

  rmw_ret_t get_service_names_and_types(
    rcutils_allocator_t * allocator,
    rmw_names_and_types_t * service_names_and_types) const;

  rmw_ret_t count_publishers(
    const char * topic_name,
    size_t * count) const;

  rmw_ret_t count_subscriptions(
    const char * topic_name,
    size_t * count) const;

  rmw_ret_t count_services(
    const char * service_name,
    size_t * count) const;

  rmw_ret_t count_clients(
    const char * service_name,
    size_t * count) const;

  rmw_ret_t get_entity_names_and_types_by_node(
    liveliness::EntityType entity_type,
    rcutils_allocator_t * allocator,
    const char * node_name,
    const char * node_namespace,
    bool no_demangle,
    rmw_names_and_types_t * names_and_types) const;

  rmw_ret_t get_entities_info_by_topic(
    liveliness::EntityType entity_type,
    rcutils_allocator_t * allocator,
    const char * topic_name,
    bool no_demangle,
    rmw_topic_endpoint_info_array_t * endpoints_info) const;

  rmw_ret_t service_server_is_available(
    const char * service_name,
    const char * service_type,
    bool * is_available);

private:
  /*
  namespace_1:
    node_1:
      enclave:
      publishers: [
          {
            topic:
            type:
            qos:
          }
      ],
      subscriptions: [
          {
            topic:
            type:
            qos:
          }
      ],
  namespace_2:
    node_n:
  */

  // We rely on a multimap to store nodes with duplicate names.
  using NodeMap = std::multimap<std::string, GraphNodePtr>;
  using NamespaceMap = std::unordered_map<std::string, NodeMap>;
  // Map namespace to a map of <node_name, GraphNodePtr>.
  NamespaceMap graph_ = {};

  // Optimize pub/sub lookups across the graph.
  GraphNode::TopicMap graph_topics_ = {};
  // Optimize service/client lookups across the graph.
  GraphNode::TopicMap graph_services_ = {};

  mutable std::mutex graph_mutex_;
};

#endif  // DETAIL__GRAPH_CACHE_HPP_
