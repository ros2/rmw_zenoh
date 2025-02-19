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

#ifndef DETAIL__RMW_CLIENT_DATA_HPP_
#define DETAIL__RMW_CLIENT_DATA_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <zenoh.hxx>

#include "event.hpp"
#include "liveliness_utils.hpp"
#include "message_type_support.hpp"
#include "service_type_support.hpp"
#include "type_support_common.hpp"
#include "zenoh_utils.hpp"

#include "rcutils/allocator.h"

#include "rmw/rmw.h"
#include "rmw/ret_types.h"

namespace rmw_zenoh_cpp
{

///=============================================================================
class ClientData final : public std::enable_shared_from_this<ClientData>
{
public:
  // Make a shared_ptr of ClientData.
  static std::shared_ptr<ClientData> make(
    std::shared_ptr<zenoh::Session> session,
    const rmw_node_t * const node,
    const rmw_client_t * client,
    liveliness::NodeInfo node_info,
    std::size_t node_id,
    std::size_t service_id,
    const std::string & service_name,
    const rosidl_service_type_support_t * type_support,
    const rmw_qos_profile_t * qos_profile);

  // Get a copy of the TopicInfo of this ClientData.
  liveliness::TopicInfo topic_info() const;

  // Returns true if liveliness token is still valid.
  bool liveliness_is_valid() const;

  // Copy the GID of this ClientData into an rmw_gid_t.
  std::array<uint8_t, 16> copy_gid() const;

  // Add a new ZenohReply to the queue.
  void add_new_reply(std::unique_ptr<rmw_zenoh_cpp::ZenohReply> reply);

  // Take a ROS service response.
  rmw_ret_t take_response(
    rmw_service_info_t * request_header,
    void * ros_response,
    bool * taken);

  // Send a service request.
  rmw_ret_t send_request(
    const void * ros_request,
    int64_t * sequence_id);

  // Set a callback to be called when events happen.
  void set_on_new_response_callback(
    rmw_event_callback_t callback,
    const void * user_data);

  // Check if there is data in the queue, and if not attach the wait set condition variable.
  bool queue_has_data_and_attach_condition_if_not(
    rmw_wait_set_data_t * wait_set_data);

  // Detach any attached wait set condition variable, and return whether there is data in the queue.
  bool detach_condition_and_queue_is_empty();

  // Shutdown this ClientData.
  rmw_ret_t shutdown();

  // Check if this ClientData is shutdown.
  bool is_shutdown() const;

  // Destructor.
  ~ClientData();

private:
  // Constructor.
  ClientData(
    const rmw_node_t * rmw_node,
    const rmw_client_t * client,
    std::shared_ptr<liveliness::Entity> entity,
    std::shared_ptr<zenoh::Session> sess,
    const void * request_type_support_impl,
    const void * response_type_support_impl,
    std::shared_ptr<RequestTypeSupport> request_type_support,
    std::shared_ptr<ResponseTypeSupport> response_type_support);

  // Internal mutex.
  mutable std::recursive_mutex mutex_;
  // The parent node.
  const rmw_node_t * rmw_node_;
  // The rmw client.
  const rmw_client_t * rmw_client_;
  // The Entity generated for the service.
  std::shared_ptr<liveliness::Entity> entity_;
  // A shared session.
  std::shared_ptr<zenoh::Session> sess_;
  // An owned keyexpression.
  std::optional<zenoh::KeyExpr> keyexpr_;
  // Liveliness token for the service.
  std::optional<zenoh::LivelinessToken> token_;
  // Type support fields.
  const void * request_type_support_impl_;
  const void * response_type_support_impl_;
  std::shared_ptr<RequestTypeSupport> request_type_support_;
  std::shared_ptr<ResponseTypeSupport> response_type_support_;
  // Deque to store the replies in the order they arrive.
  std::deque<std::unique_ptr<rmw_zenoh_cpp::ZenohReply>> reply_queue_;
  // Wait set data.
  rmw_wait_set_data_t * wait_set_data_;
  // Data callback manager.
  DataCallbackManager data_callback_mgr_;
  // Sequence number for queries.
  size_t sequence_number_;
  // Shutdown flag.
  bool is_shutdown_;
  // Whether the object has ever successfully been initialized.
  bool initialized_;
};
using ClientDataPtr = std::shared_ptr<ClientData>;
using ClientDataConstPtr = std::shared_ptr<const ClientData>;
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__RMW_CLIENT_DATA_HPP_
