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
#include <utility>

#include "rcutils/strdup.h"

#include "graph_cache.hpp"

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

PublisherData::~PublisherData()
{
  allocator_->deallocate(topic_name_, allocator_->state);
  allocator_->deallocate(node_name_, allocator_->state);
  allocator_->deallocate(namespace_name_, allocator_->state);
  allocator_->deallocate(type_name_, allocator_->state);
}

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

SubscriptionData::~SubscriptionData()
{
  allocator_->deallocate(topic_name_, allocator_->state);
  allocator_->deallocate(node_name_, allocator_->state);
  allocator_->deallocate(namespace_name_, allocator_->state);
  allocator_->deallocate(type_name_, allocator_->state);
}

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

void
GraphCache::remove_publisher(uint64_t handle)
{
  std::lock_guard<std::mutex> lck(publishers_mutex_);
  if (publishers_.count(handle) == 0) {
    return;
  }

  publishers_.erase(handle);
}

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

void
GraphCache::remove_subscription(uint64_t handle)
{
  std::lock_guard<std::mutex> lck(subscriptions_mutex_);
  if (subscriptions_.count(handle) == 0) {
    return;
  }

  subscriptions_.erase(handle);
}
