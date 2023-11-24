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

#include <zenoh.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rcutils/allocator.h"
#include "rcutils/types.h"

#include "rmw/rmw.h"
#include "rmw/names_and_types.h"


///=============================================================================
class GenerateToken
{
public:
  static std::string liveliness(size_t domain_id);

  /// Returns a string with key-expression @ros2_lv/domain_id/N/namespace/name
  static std::string node(
    size_t domain_id,
    const std::string & namespace_,
    const std::string & name);

  static std::string publisher(
    size_t domain_id,
    const std::string & node_namespace,
    const std::string & node_name,
    const std::string & topic,
    const std::string & type,
    const std::string & qos);

  static std::string subscription(
    size_t domain_id,
    const std::string & node_namespace,
    const std::string & node_name,
    const std::string & topic,
    const std::string & type,
    const std::string & qos);
};

///=============================================================================
/// Helper utilities to put/delete tokens until liveliness is supported in the
/// zenoh-c bindings.
class PublishToken
{
public:
  static bool put(
    z_owned_session_t * session,
    const std::string & token);

  static bool del(
    z_owned_session_t * session,
    const std::string & token);
};

///=============================================================================
struct TopicStats
{
  std::size_t pub_count_;
  std::size_t sub_count_;

  // Constructor which initializes counters to 0.
  TopicStats(std::size_t pub_count, std::size_t sub_count);
};
using TopicStatsPtr = std::unique_ptr<TopicStats>;

///=============================================================================
struct TopicData
{
  std::string type_;
  std::string qos_;
  TopicStats stats_;

  TopicData(
    std::string type,
    std::string qos,
    TopicStats stats);
};
using TopicDataPtr = std::shared_ptr<TopicData>;

///=============================================================================
// TODO(Yadunund): Expand to services and clients.
struct GraphNode
{
  std::string ns_;
  std::string name_;
  // TODO(Yadunund): Should enclave be the parent to the namespace key and not within a Node?
  std::string enclave_;

  // Hash topic data using "type" string to only support unique topic_types.
  // TODO(Yadunund): Should we also factor the "qos" into the cache?
  struct TopicDataHash
  {
    std::size_t operator()(const TopicDataPtr & data) const
    {
      return std::hash<std::string>{}(data->type_);
    }
  };
  // Map topic name to a set of TopicData to support multiple types per topic.
  using TopicDataSet = std::unordered_set<TopicDataPtr, TopicDataHash>;
  using TopicMap = std::unordered_map<std::string, TopicDataSet>;
  TopicMap pubs_ = {};
  TopicMap subs_ = {};
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
    rmw_names_and_types_t * topic_names_and_types);

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
  // Map namespace to a map of <node_name, GraphNodePtr>.
  std::unordered_map<std::string, std::unordered_map<std::string, GraphNodePtr>> graph_ = {};

  // Optimize topic lookups across the graph by mapping "topic_name?topic_type" keys to their pub/sub counts.
  // TODO(Yadunund): Consider storing a set of NodePtrs for each key.
  std::unordered_map<std::string, TopicStatsPtr> graph_topics_ = {};

  mutable std::mutex graph_mutex_;
};

#endif  // DETAIL__GRAPH_CACHE_HPP_
