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

#include <zenoh.h>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

#include "event.hpp"
#include "graph_cache.hpp"
#include "liveliness_utils.hpp"
#include "message_type_support.hpp"
#include "attachment_helpers.hpp"
#include "type_support_common.hpp"

#include "rcutils/allocator.h"

#include "rmw/rmw.h"
#include "rmw/ret_types.h"

namespace rmw_zenoh_cpp
{
///=============================================================================
class SubscriptionData final
{
public:
  struct Message
  {
    explicit Message(
      z_owned_slice_t p,
      uint64_t recv_ts,
      attachement_data_t && attachment);

    ~Message();

    z_owned_slice_t payload;
    uint64_t recv_timestamp;
    attachement_data_t attachment;
  };

  // Make a shared_ptr of SubscriptionData.
  static std::shared_ptr<SubscriptionData> make(
    const z_loaned_session_t * session,
    std::shared_ptr<GraphCache> graph_cache,
    const rmw_node_t * const node,
    liveliness::NodeInfo node_info,
    std::size_t node_id,
    std::size_t Subscription_id,
    const std::string & topic_name,
    const rosidl_message_type_support_t * type_support,
    const rmw_qos_profile_t * qos_profile);

  // Publish a ROS message.
  rmw_ret_t publish(
    const void * ros_message,
    std::optional<z_owned_shm_provider_t> & shm_provider);

  // Get a copy of the GUID of this SubscriptionData's liveliness::Entity.
  std::size_t guid() const;

  // Get a copy of the TopicInfo of this SubscriptionData.
  liveliness::TopicInfo topic_info() const;

  // Get the events manager of this SubscriptionData.
  std::shared_ptr<EventsManager> events_mgr() const;

  // Shutdown this SubscriptionData.
  rmw_ret_t shutdown();

  // Check if this SubscriptionData is shutdown.
  bool is_shutdown() const;

  // Add a new message to the queue.
  void add_new_message(std::unique_ptr<Message> msg, const std::string & topic_name);

  bool queue_has_data_and_attach_condition_if_not(rmw_wait_set_data_t * wait_set_data);

  bool detach_condition_and_queue_is_empty();

  rmw_ret_t take_one_message(
    void * ros_message,
    rmw_message_info_t * message_info,
    bool * taken);

  rmw_ret_t take_serialized_message(
    rmw_serialized_message_t * serialized_message,
    bool * taken,
    rmw_message_info_t * message_info);

  void set_on_new_message_callback(
    rmw_event_callback_t callback,
    const void * user_data);

  std::shared_ptr<GraphCache> graph_cache() const;

  // Destructor.
  ~SubscriptionData();

private:
  SubscriptionData(
    const rmw_node_t * rmw_node,
    std::shared_ptr<GraphCache> graph_cache,
    std::shared_ptr<liveliness::Entity> entity,
    const void * type_support_impl,
    std::unique_ptr<MessageTypeSupport> type_support);

  // Internal mutex.
  mutable std::mutex mutex_;
  // The parent node.
  const rmw_node_t * rmw_node_;
  // The graph cache.
  std::shared_ptr<GraphCache> graph_cache_;
  // The Entity generated for the subscription.
  std::shared_ptr<liveliness::Entity> entity_;
  // An owned subscriber or querying_subscriber depending on the QoS settings.
  std::variant<z_owned_subscriber_t, ze_owned_querying_subscriber_t> sub_;
  // Liveliness token for the subscription.
  zc_owned_liveliness_token_t token_;
  // Type support fields
  const void * type_support_impl_;
  std::unique_ptr<MessageTypeSupport> type_support_;
  std::deque<std::unique_ptr<Message>> message_queue_;
  // Map GID of a subscription to the sequence number of the message it published.
  std::unordered_map<size_t, int64_t> last_known_published_msg_;
  size_t total_messages_lost_;
  // Wait set data.
  rmw_wait_set_data_t * wait_set_data_;
  // std::mutex condition_mutex_;
  DataCallbackManager data_callback_mgr_;
  std::shared_ptr<EventsManager> events_mgr_;
  // Shutdown flag.
  bool is_shutdown_;
};
using SubscriptionDataPtr = std::shared_ptr<SubscriptionData>;
using SubscriptionDataConstPtr = std::shared_ptr<const SubscriptionData>;
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__RMW_SUBSCRIPTION_DATA_HPP_
