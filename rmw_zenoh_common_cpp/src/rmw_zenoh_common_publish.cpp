// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

#include "rcutils/logging_macros.h"

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/rmw.h"

#include "impl/type_support_common.hpp"
#include "impl/pubsub_impl.hpp"

#include "rmw_zenoh_common_cpp/rmw_zenoh_common.h"
#include "rmw_zenoh_common_cpp/zenoh-net-interface.h"

/// PUBLISH ROS MESSAGE ========================================================
// Serialize and publish a ROS message using Zenoh.
rmw_ret_t
rmw_zenoh_common_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation,
  const char * const eclipse_zenoh_identifier)
{
  (void) allocation;
  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_common_cpp", "[rmw_publish] %s (%ld)",
    publisher->topic_name,
    static_cast<rmw_publisher_data_t *>(publisher->data)->zn_topic_id_);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto publisher_data = static_cast<rmw_publisher_data_t *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_data, RMW_RET_ERROR);

  // ASSIGN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator =
    &(static_cast<rmw_publisher_data_t *>(publisher->data)->node_->context->options.allocator);

  // SERIALIZE DATA ============================================================
  size_t max_data_length = (static_cast<rmw_publisher_data_t *>(publisher->data)
    ->type_support_->getEstimatedSerializedSize(ros_message));

  // Init serialized message byte array
  uint8_t * msg_buffer = static_cast<uint8_t *>(allocator->allocate(max_data_length, allocator->state));

  // Object that manages the raw buffer
  ucdrBuffer writer;
  ucdr_init_buffer(&writer, msg_buffer, max_data_length);

  // Object that serializes the data
  if (!publisher_data->type_support_->serializeROSmessage(
      ros_message,
      &writer,
      publisher_data->type_support_impl_))
  {
    RMW_SET_ERROR_MSG("could not serialize ROS message");
    allocator->deallocate(msg_buffer, allocator->state);
    return RMW_RET_ERROR;
  }

  size_t data_length = ucdr_buffer_length(&writer);

  // PUBLISH ON ZENOH MIDDLEWARE LAYER =========================================
  size_t wrid_ret = zn_write(
    publisher_data->zn_session_,
    zn_rid(publisher_data->zn_topic_id_),
    reinterpret_cast<char *>(msg_buffer),
    data_length);

  allocator->deallocate(msg_buffer, allocator->state);

  if (wrid_ret == 0) {
    return RMW_RET_OK;
  } else {
    RMW_SET_ERROR_MSG("zenoh failed to publish response");
    return RMW_RET_ERROR;
  }
}

rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)serialized_message;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_common_cpp", "rmw_publish_serialized_message");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)ros_message;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_common_cpp", "rmw_publish_loaned_message");
  return RMW_RET_UNSUPPORTED;
}
