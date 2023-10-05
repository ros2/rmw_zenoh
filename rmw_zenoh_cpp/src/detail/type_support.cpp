// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

// This file is originally from:
// https://github.com/ros2/rmw_fastrtps/blob/469624e3d483290d6f88fe4b89ee5feaa7694e61/rmw_fastrtps_cpp/src/type_support_common.hpp

#include <cassert>
#include <functional>
#include <memory>

#include "fastrtps/types/DynamicData.h"
#include "fastrtps/types/DynamicPubSubType.h"

#include "rmw/error_handling.h"

#include "type_support.hpp"

TypeSupport::TypeSupport()
{
  m_isGetKeyDefined = false;
  max_size_bound_ = false;
  is_plain_ = false;
  auto_fill_type_object(false);
  auto_fill_type_information(false);
}

void TypeSupport::set_members(const message_type_support_callbacks_t * members)
{
  members_ = members;

#ifdef ROSIDL_TYPESUPPORT_FASTRTPS_HAS_PLAIN_TYPES
  char bounds_info;
  auto data_size = static_cast<uint32_t>(members->max_serialized_size(bounds_info));
  max_size_bound_ = 0 != (bounds_info & ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE);
  is_plain_ = bounds_info == ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE;
#else
  is_plain_ = true;
  auto data_size = static_cast<uint32_t>(members->max_serialized_size(is_plain_));
  max_size_bound_ = is_plain_;
#endif

  // A plain message of size 0 is an empty message
  if (is_plain_ && (data_size == 0) ) {
    has_data_ = false;
    ++data_size;  // Dummy byte
  } else {
    has_data_ = true;
  }

  // Total size is encapsulation size + data size
  m_typeSize = 4 + data_size;
  // Account for RTPS submessage alignment
  m_typeSize = (m_typeSize + 3) & ~3;
}

size_t TypeSupport::getEstimatedSerializedSize(const void * ros_message, const void * impl) const
{
  if (is_plain_) {
    return m_typeSize;
  }

  assert(ros_message);
  assert(impl);

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);

  // Encapsulation size + message size
  return 4 + callbacks->get_serialized_size(ros_message);
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

bool TypeSupport::deserializeROSmessage(
  eprosima::fastcdr::Cdr & deser,
  void * ros_message,
  const void * impl) const
{
  assert(ros_message);
  assert(impl);

  try {
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
  } catch (const eprosima::fastcdr::exception::Exception &) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Fast CDR exception deserializing message of type %s.",
      getName());
    return false;
  }

  return true;
}

void TypeSupport::deleteData(void * data)
{
  assert(data);
  delete static_cast<eprosima::fastcdr::FastBuffer *>(data);
}

void * TypeSupport::createData()
{
  return new eprosima::fastcdr::FastBuffer();
}

bool TypeSupport::serialize(
  void * data, eprosima::fastrtps::rtps::SerializedPayload_t * payload)
{
  assert(data);
  assert(payload);

  auto ser_data = static_cast<SerializedData *>(data);

  switch (ser_data->type) {
    case FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE:
      {
        eprosima::fastcdr::FastBuffer fastbuffer(  // Object that manages the raw buffer
          reinterpret_cast<char *>(payload->data), payload->max_size);
        eprosima::fastcdr::Cdr ser(  // Object that serializes the data
          fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
        if (this->serializeROSmessage(ser_data->data, ser, ser_data->impl)) {
          payload->encapsulation = ser.endianness() ==
            eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
          payload->length = (uint32_t)ser.getSerializedDataLength();
          return true;
        }
        break;
      }

    case FASTRTPS_SERIALIZED_DATA_TYPE_CDR_BUFFER:
      {
        auto ser = static_cast<eprosima::fastcdr::Cdr *>(ser_data->data);
        if (payload->max_size >= ser->getSerializedDataLength()) {
          payload->length = static_cast<uint32_t>(ser->getSerializedDataLength());
          payload->encapsulation = ser->endianness() ==
            eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
          memcpy(payload->data, ser->getBufferPointer(), ser->getSerializedDataLength());
          return true;
        }
        break;
      }

    case FASTRTPS_SERIALIZED_DATA_TYPE_DYNAMIC_MESSAGE:
      {
        auto m_type = std::make_shared<eprosima::fastrtps::types::DynamicPubSubType>();

        // Serializes payload into dynamic data stored in data->data
        return m_type->serialize(
          static_cast<eprosima::fastrtps::types::DynamicData *>(ser_data->data), payload
        );
      }

    default:
      return false;
  }
  return false;
}

bool TypeSupport::deserialize(
  eprosima::fastrtps::rtps::SerializedPayload_t * payload,
  void * data)
{
  assert(data);
  assert(payload);

  auto ser_data = static_cast<SerializedData *>(data);

  switch (ser_data->type) {
    case FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE:
      {
        eprosima::fastcdr::FastBuffer fastbuffer(
          reinterpret_cast<char *>(payload->data), payload->length);
        eprosima::fastcdr::Cdr deser(
          fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
        return deserializeROSmessage(deser, ser_data->data, ser_data->impl);
      }

    case FASTRTPS_SERIALIZED_DATA_TYPE_CDR_BUFFER:
      {
        auto buffer = static_cast<eprosima::fastcdr::FastBuffer *>(ser_data->data);
        if (!buffer->reserve(payload->length)) {
          return false;
        }
        memcpy(buffer->getBuffer(), payload->data, payload->length);
        return true;
      }

    case FASTRTPS_SERIALIZED_DATA_TYPE_DYNAMIC_MESSAGE:
      {
        auto m_type = std::make_shared<eprosima::fastrtps::types::DynamicPubSubType>();

        // Deserializes payload into dynamic data stored in data->data (copies!)
        return m_type->deserialize(
          payload, static_cast<eprosima::fastrtps::types::DynamicData *>(ser_data->data)
        );
      }

    default:
      return false;
  }
  return false;
}

std::function<uint32_t()> TypeSupport::getSerializedSizeProvider(void * data)
{
  assert(data);

  auto ser_data = static_cast<SerializedData *>(data);
  auto ser_size = [this, ser_data]() -> uint32_t
    {
      if (ser_data->type == FASTRTPS_SERIALIZED_DATA_TYPE_CDR_BUFFER) {
        auto ser = static_cast<eprosima::fastcdr::Cdr *>(ser_data->data);
        return static_cast<uint32_t>(ser->getSerializedDataLength());
      }
      return static_cast<uint32_t>(
        this->getEstimatedSerializedSize(ser_data->data, ser_data->impl));
    };
  return ser_size;
}
