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

#include <zenoh.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

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
class ClientData final
{
public:
  // Make a shared_ptr of ClientData.
  static std::shared_ptr<ClientData> make(
    z_session_t session,
    const rmw_node_t * const node,
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
  void copy_gid(rmw_gid_t * gid) const;

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

  void set_on_new_response_callback(
    rmw_event_callback_t callback,
    const void * user_data);

  // rmw_wait helpers.
  bool queue_has_data_and_attach_condition_if_not(
    rmw_wait_set_data_t * wait_set_data);

  bool detach_condition_and_queue_is_empty();

  // See the comment for "num_in_flight" below on the use of this method.
  bool decrement_queries_in_flight_and_is_shutdown(bool & queries_in_flight);

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
    std::shared_ptr<liveliness::Entity> entity,
    const void * request_type_support_impl,
    const void * response_type_support_impl,
    std::unique_ptr<RequestTypeSupport> request_type_support,
    std::unique_ptr<ResponseTypeSupport> response_type_support);

  // Internal mutex.
  mutable std::mutex mutex_;
  // The parent node.
  const rmw_node_t * rmw_node_;
  // The Entity generated for the service.
  std::shared_ptr<liveliness::Entity> entity_;
  // The GID for this client.
  uint8_t gid_[RMW_GID_STORAGE_SIZE];
  // An owned keyexpression.
  z_owned_keyexpr_t keyexpr_;
  // Liveliness token for the service.
  zc_owned_liveliness_token_t token_;
  // Type support fields.
  const void * request_type_support_impl_;
  const void * response_type_support_impl_;
  std::unique_ptr<RequestTypeSupport> request_type_support_;
  std::unique_ptr<ResponseTypeSupport> response_type_support_;
  // Deque to store the replies in the order they arrive.
  std::deque<std::unique_ptr<rmw_zenoh_cpp::ZenohReply>> reply_queue_;
  // Wait set data.
  rmw_wait_set_data_t * wait_set_data_;
  // Data callback manager.
  DataCallbackManager data_callback_mgr_;
  // Sequence number for queries.
  size_t sequence_number_;
  // rmw_zenoh uses Zenoh queries to implement clients. It turns out that in Zenoh, there is no
  // way to cancel a query once it is in-flight via the z_get() zenoh-c API. Thus, if an
  // rmw_zenoh_cpp user does rmw_create_client(), rmw_send_request(), rmw_destroy_client(), but the
  // query comes in after the rmw_destroy_client(), rmw_zenoh_cpp could access already-freed memory.
  // The next 2 variables are used to avoid that situation. Any time a query is initiated via
  // rmw_send_request(), num_in_flight_ is incremented.  When the Zenoh calls the callback for the
  // query reply, num_in_flight_ is decremented.
  // When shutdown() is called, is_shutdown_ is set to true.
  // If num_in_flight_ is 0, the data associated with this structure is freed.
  // If num_in_flight_ is *not* 0, then the data associated with this structure is maintained.
  // In the situation where is_shutdown_ is true, and num_in_flight_ drops to 0 in the query
  // callback, the query callback will free up the structure.
  //
  // There is one case which is not handled by this, which has to do with timeouts.  The query
  // timeout is currently set to essentially infinite.  Thus, if a query is in-flight but never
  // returns, the memory in this structure will never be freed.  There isn't much we can do about
  // that at this time, but we may want to consider changing the timeout so that the memory can
  // eventually be freed up.
  size_t num_in_flight_;
  // Shutdown flag.
  bool is_shutdown_;
};
using ClientDataPtr = std::shared_ptr<ClientData>;
using ClientDataConstPtr = std::shared_ptr<const ClientData>;
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__RMW_CLIENT_DATA_HPP_
