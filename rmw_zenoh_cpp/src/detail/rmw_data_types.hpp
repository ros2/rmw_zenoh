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

#ifndef DETAIL__RMW_DATA_TYPES_HPP_
#define DETAIL__RMW_DATA_TYPES_HPP_

#include <zenoh.h>

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "rcutils/allocator.h"

#include "rmw/rmw.h"

#include "rosidl_runtime_c/type_hash.h"

#include "event.hpp"
#include "message_type_support.hpp"
#include "rmw_wait_set_data.hpp"
#include "service_type_support.hpp"
#include "zenoh_utils.hpp"

/// Structs for various type erased data fields.
namespace rmw_zenoh_cpp
{
///=============================================================================
void client_data_handler(z_owned_reply_t * reply, void * client_data);
void client_data_drop(void * data);

///=============================================================================
class rmw_client_data_t final
{
public:
  // The Entity generated for the client.
  std::shared_ptr<liveliness::Entity> entity;

  z_owned_keyexpr_t keyexpr;

  // Store the actual QoS profile used to configure this client.
  // The QoS is reused for sending requests and getting responses.
  rmw_qos_profile_t adapted_qos_profile;

  // Liveliness token for the client.
  zc_owned_liveliness_token_t token;

  const void * request_type_support_impl;
  const void * response_type_support_impl;
  const char * typesupport_identifier;
  const rosidl_type_hash_t * type_hash;
  RequestTypeSupport * request_type_support;
  ResponseTypeSupport * response_type_support;

  rmw_context_t * context;

  size_t get_next_sequence_number();

  void add_new_reply(std::unique_ptr<rmw_zenoh_cpp::ZenohReply> reply);

  bool queue_has_data_and_attach_condition_if_not(rmw_wait_set_data_t * wait_set_data);

  bool detach_condition_and_queue_is_empty();

  std::unique_ptr<rmw_zenoh_cpp::ZenohReply> pop_next_reply();

  DataCallbackManager data_callback_mgr;

  // See the comment for "num_in_flight" below on the use of this method.
  void increment_in_flight_callbacks();

  // See the comment for "num_in_flight" below on the use of this method.
  bool shutdown_and_query_in_flight();

  // See the comment for "num_in_flight" below on the use of this method.
  bool decrement_queries_in_flight_and_is_shutdown(bool & queries_in_flight);

  bool is_shutdown() const;

private:
  void notify();

  size_t sequence_number_{1};
  std::mutex sequence_number_mutex_;

  rmw_wait_set_data_t * wait_set_data_{nullptr};
  std::mutex condition_mutex_;

  std::deque<std::unique_ptr<rmw_zenoh_cpp::ZenohReply>> reply_queue_;
  mutable std::mutex reply_queue_mutex_;

  // rmw_zenoh uses Zenoh queries to implement clients.  It turns out that in Zenoh, there is no
  // way to cancel a query once it is in-flight via the z_get() zenoh-c API. Thus, if an
  // rmw_zenoh_cpp user does rmw_create_client(), rmw_send_request(), rmw_destroy_client(), but the
  // query comes in after the rmw_destroy_client(), rmw_zenoh_cpp could access already-freed memory.
  //
  // The next 3 variables are used to avoid that situation.  Any time a query is initiated via
  // rmw_send_request(), num_in_flight_ is incremented.  When the Zenoh calls the callback for the
  // query reply, num_in_flight_ is decremented.  When rmw_destroy_client() is called, is_shutdown_
  // is set to true.  If num_in_flight_ is 0, the data associated with this structure is freed.
  // If num_in_flight_ is *not* 0, then the data associated with this structure is maintained.
  // In the situation where is_shutdown_ is true, and num_in_flight_ drops to 0 in the query
  // callback, the query callback will free up the structure.
  //
  // There is one case which is not handled by this, which has to do with timeouts.  The query
  // timeout is currently set to essentially infinite.  Thus, if a query is in-flight but never
  // returns, the memory in this structure will never be freed.  There isn't much we can do about
  // that at this time, but we may want to consider changing the timeout so that the memory can
  // eventually be freed up.
  mutable std::mutex in_flight_mutex_;
  bool is_shutdown_{false};
  size_t num_in_flight_{0};
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__RMW_DATA_TYPES_HPP_
