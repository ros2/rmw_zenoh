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

#include "rcutils/allocator.h"

class PublisherData final
{
public:
  PublisherData(
    const char * topic, const char * node, const char * namespace_,
    const char * type, rcutils_allocator_t * allocator);

  ~PublisherData();

private:
  rcutils_allocator_t * allocator_;
  char * topic_name_{nullptr};
  char * node_name_{nullptr};
  char * namespace_name_{nullptr};
  char * type_name_{nullptr};
};

class SubscriptionData final
{
public:
  SubscriptionData(
    const char * topic, const char * node, const char * namespace_,
    const char * type, rcutils_allocator_t * allocator);

  ~SubscriptionData();

private:
  rcutils_allocator_t * allocator_;
  char * topic_name_{nullptr};
  char * node_name_{nullptr};
  char * namespace_name_{nullptr};
  char * type_name_{nullptr};
};

class GraphCache final
{
public:
  uint64_t
  add_publisher(
    const char * topic, const char * node, const char * namespace_,
    const char * type, rcutils_allocator_t * allocator);

  void remove_publisher(uint64_t publisher_handle);

  uint64_t
  add_subscription(
    const char * topic, const char * node, const char * namespace_,
    const char * type, rcutils_allocator_t * allocator);

  void remove_subscription(uint64_t subscription_handle);

private:
  std::mutex publishers_mutex_;
  uint64_t publishers_handle_id_{0};
  std::map<uint64_t, std::unique_ptr<PublisherData>> publishers_;

  std::mutex subscriptions_mutex_;
  uint64_t subscriptions_handle_id_{0};
  std::map<uint64_t, std::unique_ptr<SubscriptionData>> subscriptions_;
};

#endif  // DETAIL__GRAPH_CACHE_HPP_
