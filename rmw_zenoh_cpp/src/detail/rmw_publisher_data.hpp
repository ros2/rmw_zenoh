// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#ifndef DETAIL__RMW_PUBLISHER_DATA_HPP_
#define DETAIL__RMW_PUBLISHER_DATA_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <zenoh.hxx>

#include "endpoint_info.hpp"
#include "event.hpp"
#include "graph_cache.hpp"
#include "liveliness_utils.hpp"
#include "message_type_support.hpp"
#include "type_support_common.hpp"
#include "zenoh_utils.hpp"

#include "rcutils/allocator.h"

#include "rmw/rmw.h"
#include "rmw/ret_types.h"

namespace rmw_zenoh_cpp
{
///=============================================================================
class PublisherData final
{
public:
  // Make a shared_ptr of PublisherData.
  static std::shared_ptr<PublisherData> make(
    std::shared_ptr<zenoh::Session> session,
    const rmw_publisher_t * const rmw_publisher,
    const rmw_node_t * const node,
    liveliness::NodeInfo node_info,
    std::size_t node_id,
    std::size_t publisher_id,
    const std::string & topic_name,
    const rosidl_message_type_support_t * type_support,
    const rmw_qos_profile_t * qos_profile);

  // Publish a ROS message.
  rmw_ret_t publish(
    const void *ros_message,
    ShmContext *shm
  );

  // Publish a serialized ROS message.
  rmw_ret_t publish_serialized_message(
    const rmw_serialized_message_t *serialized_message,
    ShmContext *shm
  );

  // Get a copy of the gid_hash of this PublisherData's liveliness::Entity.
  std::size_t gid_hash() const;

  // Get a copy of the TopicInfo of this PublisherData.
  liveliness::TopicInfo topic_info() const;

  // Return a copy of the GID of this publisher.
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> copy_gid() const;

  // Returns true if liveliness token is still valid.
  bool liveliness_is_valid() const;

  // Get the events manager of this PublisherData.
  std::shared_ptr<EventsManager> events_mgr() const;

  // Shutdown this PublisherData.
  rmw_ret_t shutdown();

  // Check if this PublisherData is shutdown.
  bool is_shutdown() const;

  // Destructor.
  ~PublisherData();

private:
  // Structures for Buffer-aware publishers
  struct SubscriberInfo
  {
    rmw_gid_t gid;
    std::string endpoint_key;
    EndpointInfoStorage endpoint_info;
    bool uses_cpu_group{false};
    std::unordered_map<std::string, std::string> backend_metadata;
    std::unordered_map<std::string, std::vector<std::set<uint32_t>>> backend_groups;
  };

  struct PublisherEndpoint
  {
    std::string key;
    std::optional<zenoh::ext::AdvancedPublisher> pub;
    std::vector<rmw_gid_t> target_subscribers;
    std::optional<std::vector<uint8_t>> cached_message;
  };

  // Constructor.
  PublisherData(
    const rmw_publisher_t * const rmw_publisher,
    const rmw_node_t * rmw_node,
    std::shared_ptr<liveliness::Entity> entity,
    std::shared_ptr<zenoh::Session> session,
    zenoh::LivelinessToken token,
    const void * type_support_impl,
    std::unique_ptr<MessageTypeSupport> type_support,
    bool is_buffer_aware,
    std::unordered_map<std::string, std::string> backend_metadata);

  // Discovery callback for Buffer-aware publishers
  void on_subscriber_discovered(const liveliness::Entity & entity);

  // Get or create an endpoint for a specific full key (caller must hold mutex_)
  std::shared_ptr<PublisherEndpoint> get_or_create_endpoint(
    const std::string & full_key);

  // Create a Zenoh publisher endpoint (does not require mutex_).
  // buffer_aware=false builds the base publisher (with sample-miss
  // detection); buffer_aware=true builds a per-subscriber buffer-aware endpoint.
  std::shared_ptr<PublisherEndpoint> create_publisher_endpoint(
    const std::string & full_key, bool buffer_aware = true);

  // Buffer-aware publish helper
  rmw_ret_t publish_buffer_aware(
    const void * ros_message,
    ShmContext * shm);

  // Internal mutex.
  mutable std::mutex mutex_;
  // The rmw publisher
  const rmw_publisher_t * rmw_publisher_;
  // The parent node.
  const rmw_node_t * rmw_node_;
  // The Entity generated for the publisher.
  std::shared_ptr<liveliness::Entity> entity_;
  // A shared session.
  std::shared_ptr<zenoh::Session> sess_;
  // Non-owning cache of the base publisher, which is stored in endpoints_
  // under the topic key expression. Lets publish() reach it without a per-message
  // map lookup. Owned by endpoints_; cleared on shutdown.
  PublisherEndpoint * base_endpoint_{nullptr};
  // Liveliness token for the publisher.
  std::optional<zenoh::LivelinessToken> token_;
  // Type support fields
  const void * type_support_impl_;
  std::unique_ptr<MessageTypeSupport> type_support_;
  std::shared_ptr<EventsManager> events_mgr_;
  size_t sequence_number_;
  // Shutdown flag.
  bool is_shutdown_;

  // Buffer-aware publisher fields
  bool is_buffer_aware_;
  std::unordered_map<std::string, std::string> backend_metadata_;
  // All publisher endpoints keyed by full key: the base publisher under the
  // topic key expression, plus buffer-aware endpoints added on subscriber discovery.
  std::unordered_map<std::string, std::shared_ptr<PublisherEndpoint>> endpoints_;
  std::set<std::string> pending_endpoints_;
  std::vector<SubscriberInfo> discovered_subscribers_;
  EndpointInfoStorage local_endpoint_info_;
  std::shared_ptr<GraphCache> graph_cache_;
};
using PublisherDataPtr = std::shared_ptr<PublisherData>;
using PublisherDataConstPtr = std::shared_ptr<const PublisherData>;
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__RMW_PUBLISHER_DATA_HPP_
