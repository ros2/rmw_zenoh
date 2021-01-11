// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

// This file is originally from:
// https://github.com/ros2/rmw_fastrtps/blob/0134bcf52244cbbb6e7e8ac631de391beaa85f17/rmw_fastrtps_cpp/src/type_support_common.cpp

#include <string>

#include "rmw/error_handling.h"
#include "type_support_common.hpp"

namespace rmw_zenoh_common_cpp
{

TypeSupport::TypeSupport()
{
  max_size_bound_ = false;
}

void TypeSupport::set_members(const message_type_support_callbacks_t * members)
{
  members_ = members;

  // Fully bound by default
  max_size_bound_ = true;
  auto data_size = static_cast<uint32_t>(members->max_serialized_size(max_size_bound_));

  // A fully bound message of size 0 is an empty message
  if (max_size_bound_ && (data_size == 0) ) {
    has_data_ = false;
    ++data_size;  // Dummy byte
  } else {
    has_data_ = true;
  }

  // Total size is encapsulation size + data size
  type_size_ = 4 + data_size;
}

size_t TypeSupport::getEstimatedSerializedSize(const void * ros_message)
{
  if (max_size_bound_) {
    return type_size_;
  }

  assert(ros_message);

  // Encapsulation size + message size
  return 4 + members_->get_serialized_size(ros_message);
}

bool TypeSupport::serializeROSmessage(
  const void * ros_message,
  eprosima::fastcdr::Cdr & ser,
  const void * impl) const
{
  assert(ros_message);
  assert(impl);

  // Serialize encapsulation
  ser.serialize_encapsulation();

  // If type is not empty, serialize message
  if (has_data_) {
    auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);
    return callbacks->cdr_serialize(ros_message, ser);
  }

  // Otherwise, add a dummy byte
  ser << (uint8_t)0;
  return true;
}

bool TypeSupport::serializeROSmessage(
  const void * ros_message,
  ucdrBuffer * writer,
  const void * impl) const
{
  assert(ros_message);
  assert(impl);

  // If type is not empty, serialize message
  if (has_data_) {
    auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);
    return callbacks->cdr_serialize_ucdr(ros_message, writer);
  }

  // Otherwise, add a dummy byte
  ucdr_serialize_uint8_t(writer, 0);
  return true;
}

bool TypeSupport::deserializeROSmessage(
  eprosima::fastcdr::Cdr & deser,
  void * ros_message,
  const void * impl) const
{
  assert(ros_message);
  assert(impl);

  // Deserialize encapsulation.
  deser.read_encapsulation();

  // If type is not empty, deserialize message
  if (has_data_) {
    auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);
    return callbacks->cdr_deserialize(deser, ros_message);
  }

  // Otherwise, consume dummy byte
  uint8_t dump = 0;
  deser >> dump;
  (void)dump;

  return true;
}

MessageTypeSupport::MessageTypeSupport(const message_type_support_callbacks_t * members)
{
  assert(members);

  set_members(members);
}

ServiceTypeSupport::ServiceTypeSupport()
{
}

RequestTypeSupport::RequestTypeSupport(const service_type_support_callbacks_t * members)
{
  assert(members);

  auto msg = static_cast<const message_type_support_callbacks_t *>(
    members->request_members_->data);

  set_members(msg);
}

ResponseTypeSupport::ResponseTypeSupport(const service_type_support_callbacks_t * members)
{
  assert(members);

  auto msg = static_cast<const message_type_support_callbacks_t *>(
    members->response_members_->data);

  set_members(msg);
}

}  // namespace rmw_zenoh_common_cpp
