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

#ifndef DETAIL__RMW_SUBSCRIPTION_DATA_HPP_
#define DETAIL__RMW_SUBSCRIPTION_DATA_HPP_

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <array>

#include <zenoh.hxx>

#include "attachment_helpers.hpp"
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
class SubscriptionData final : public std::enable_shared_from_this<SubscriptionData>
{
public:
  struct Message
  {
    Message(
      const zenoh::Bytes & p,
      uint64_t recv_ts,
      AttachmentData && attachment_,
      std::optional<EndpointInfoStorage> endpoint_info_ = std::nullopt);

    Payload payload;
    uint64_t recv_timestamp;
    AttachmentData attachment;
    std::optional<EndpointInfoStorage> endpoint_info;
  };

  // Make a shared_ptr of SubscriptionData.
  static std::shared_ptr<SubscriptionData> make(
    std::shared_ptr<zenoh::Session> session,
    std::shared_ptr<GraphCache> graph_cache,
    const rmw_node_t * const node,
    liveliness::NodeInfo node_info,
    std::size_t node_id,
    std::size_t subscription_id,
    const std::string & topic_name,
    const rosidl_message_type_support_t * type_support,
    const rmw_qos_profile_t * qos_profile,
    const rmw_subscription_options_t & sub_options);

  // Get a copy of the gid_hash of this SubscriptionData's liveliness::Entity.
  std::size_t gid_hash() const;

  // Get a copy of the TopicInfo of this SubscriptionData.
  liveliness::TopicInfo topic_info() const;

  // Returns true if liveliness token is still valid.
  bool liveliness_is_valid() const;

  // Get the events manager of this SubscriptionData.
  std::shared_ptr<EventsManager> events_mgr() const;

  // Add a new message to the queue.
  void add_new_message(
    std::unique_ptr<Message> msg,
    const std::string & topic_name);

  // Take a ROS message.
  rmw_ret_t take_one_message(
    void * ros_message,
    rmw_message_info_t * message_info,
    bool * taken);

  // Take a serialized ROS message.
  rmw_ret_t take_serialized_message(
    rmw_serialized_message_t * serialized_message,
    bool * taken,
    rmw_message_info_t * message_info);

  void set_on_new_message_callback(
    rmw_event_callback_t callback,
    const void * user_data);

  bool queue_has_data_and_attach_condition_if_not(
    rmw_wait_set_data_t * wait_set_data);

  bool detach_condition_and_queue_is_empty();

  // Return graph cache for event registrations.
  std::shared_ptr<GraphCache> graph_cache() const;

  // Shutdown this SubscriptionData.
  rmw_ret_t shutdown();

  // Check if this SubscriptionData is shutdown.
  bool is_shutdown() const;

  // Destructor.
  ~SubscriptionData();

private:
  struct PublisherInfo
  {
    rmw_gid_t gid{};
    std::string endpoint_key;
    EndpointInfoStorage endpoint_info;
    std::unordered_map<std::string, std::string> backend_metadata;
    std::unordered_map<std::string, std::vector<std::set<uint32_t>>> backend_groups;
  };

  struct SubscriptionEndpoint
  {
    std::string key;
    std::optional<EndpointInfoStorage> publisher_info;
    std::optional<zenoh::ext::AdvancedSubscriber<void>> sub;
    // When true, this endpoint is a shared accelerated channel keyed by the
    // local subscriber's GID; any buffer-aware publisher matched to this
    // subscriber writes to it. The message callback must resolve the sending
    // publisher's endpoint_info by GID lookup against discovered_publishers_.
    bool is_accelerated{false};
  };

  // Constructor.
  SubscriptionData(
    const rmw_node_t * rmw_node,
    std::shared_ptr<GraphCache> graph_cache,
    std::shared_ptr<liveliness::Entity> entity,
    std::shared_ptr<zenoh::Session> session,
    const void * type_support_impl,
    std::unique_ptr<MessageTypeSupport> type_support,
    rmw_subscription_options_t sub_options,
    bool is_buffer_aware,
    std::vector<std::string> my_backend_types);

  // Initialize the subscription.
  bool init();

  // Discovery callback for Buffer-aware subscriptions.
  void on_publisher_discovered(const liveliness::Entity & entity);

  // Create a subscription endpoint for a specific key (caller must hold mutex_).
  void create_subscription_for_key(
    const std::string & key,
    std::optional<EndpointInfoStorage> publisher_info);

  // Create a Zenoh subscription endpoint (does not require mutex_).
  std::shared_ptr<SubscriptionEndpoint> create_subscription_endpoint(
    const std::string & key,
    std::optional<EndpointInfoStorage> publisher_info,
    bool is_accelerated);

  // Look up a discovered publisher's endpoint info by GID.  Used by the
  // shared accelerated channel to attach endpoint-aware context to each
  // received message.  Thread-safe: acquires mutex_ internally.
  std::optional<EndpointInfoStorage> lookup_publisher_endpoint_info(
    const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & publisher_gid) const;

  // Internal mutex.
  mutable std::mutex mutex_;
  // The parent node.
  const rmw_node_t * rmw_node_;
  // Graph cache for discovery.
  std::shared_ptr<GraphCache> graph_cache_;
  // The Entity generated for the subscription.
  std::shared_ptr<liveliness::Entity> entity_;
  // A shared session.
  std::shared_ptr<zenoh::Session> sess_;
  // An owned AdvancedSubscriber for non-Buffer-aware subscriptions.
  std::optional<zenoh::ext::AdvancedSubscriber<void>> sub_;
  // Liveliness token for the subscription.
  std::optional<zenoh::LivelinessToken> token_;
  // Type support fields.
  const void * type_support_impl_;
  std::unique_ptr<MessageTypeSupport> type_support_;
  rmw_subscription_options_t sub_options_;
  std::shared_ptr<EventsManager> events_mgr_;
  // Message queue.
  std::deque<std::unique_ptr<Message>> message_queue_;
  // Map of publisher gid hash to last known sequence number.
  std::unordered_map<size_t, int64_t> last_known_published_msg_;
  // Per-subscriber reception sequence number counter, incremented on every take.
  std::atomic<uint64_t> reception_sn_;
  // Wait set data.
  rmw_wait_set_data_t * wait_set_data_;
  // Data callback manager.
  DataCallbackManager data_callback_mgr_;
  // Shutdown flag.
  bool is_shutdown_;
  // Whether the object has ever successfully been initialized.
  bool initialized_;

  // Buffer-aware subscription fields
  bool is_buffer_aware_;
  std::vector<std::string> my_backend_types_;
  EndpointInfoStorage local_endpoint_info_;
  std::vector<PublisherInfo> discovered_publishers_;
  std::unordered_map<std::string, std::shared_ptr<SubscriptionEndpoint>> sub_endpoints_;
  std::set<std::string> pending_sub_endpoints_;
};
using SubscriptionDataPtr = std::shared_ptr<SubscriptionData>;
using SubscriptionDataConstPtr = std::shared_ptr<const SubscriptionData>;
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__RMW_SUBSCRIPTION_DATA_HPP_
