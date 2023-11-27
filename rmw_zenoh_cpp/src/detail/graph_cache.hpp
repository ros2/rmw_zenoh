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
#include "rmw/names_and_types.h"


///=============================================================================
struct TopicStats
{
  std::size_t pub_count_;
  std::size_t sub_count_;

  // Constructor which initializes counters to 0.
  TopicStats(std::size_t pub_count, std::size_t sub_count);
};
using TopicStatsPtr = std::unique_ptr<TopicStats>;
using TopicInfo = liveliness::TopicInfo;

///=============================================================================
struct TopicData
{
  TopicInfo info_;
  TopicStats stats_;

  TopicData(
    TopicInfo info,
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

  // Map topic type to TopicData
  using TopicDataMap = std::unordered_map<std::string, TopicDataPtr>;
  // Map topic name to TopicDataMap
  using TopicMap = std::unordered_map<std::string, TopicDataMap>;
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
