// Copyright 2020 Continental AG
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

#include "serializer_c.hpp"

#include <string>
#include <stdexcept>

#include <rosidl_typesupport_introspection_c/field_types.h>

#include "internal/rosidl_generator_c_pkg_adapter.hpp"
#include "internal/common.hpp"

namespace eCAL
{
  namespace rmw
  {

    template <typename T>
    void CSerializer::SerializeSingle(const char* data, std::string& serialized_data) const
    {
      serialized_data.insert(serialized_data.end(), data, data + sizeof(T));
    }

    template <typename T>
    void CSerializer::SerializeSingle(const T& data, std::string& serialized_data) const
    {
      auto data_bytes = reinterpret_cast<const char*>(&data);
      SerializeSingle<T>(data_bytes, serialized_data);
    }

    template <>
    void CSerializer::SerializeSingle<std::string>(const char* data, std::string& serialized_data) const
    {
      SerializeDynamicArray<char>(data, serialized_data);
    }

    template <typename T>
    void CSerializer::SerializeArray(const char* data, size_t count, std::string& serialized_data) const
    {
      serialized_data.insert(serialized_data.end(), data, data + count * sizeof(T));
      // TODO: PVS issue V1001 (https://www.viva64.com/en/w/v1001/print/)
      // what is this line supposed to do ?
      data += sizeof(T) * count;
    }

    template <>
    void CSerializer::SerializeArray<std::string>(const char* data, size_t count, std::string& serialized_data) const
    {
      for (size_t i = 0; i < count; i++)
      {
        SerializeSingle<std::string>(data, serialized_data);
        data += sizeof(rosidl_runtime_c__char__Sequence);
      }
    }

    template <>
    void CSerializer::SerializeArray<ros_message_t>(const char* data,
                                                    const rosidl_typesupport_introspection_c__MessageMember* member,
                                                    std::string& serialized_data) const
    {
      auto sub_members = GetMembers(member);
      for (size_t i = 0; i < member->array_size_; i++)
      {
        SerializeMessage(data, sub_members, serialized_data);
        data += sub_members->size_of_;
      }
    }

    template <typename T>
    void CSerializer::SerializeDynamicArray(const char* data, std::string& serialized_data) const
    {
      auto sequence = reinterpret_cast<const rosidl_runtime_c__char__Sequence*>(data);

      serialized_data.reserve(serialized_data.size() + sequence->size * sizeof(T) + sizeof(array_size_t));
      SerializeSingle<array_size_t>(sequence->size, serialized_data);
      SerializeArray<T>(reinterpret_cast<const char*>(sequence->data), sequence->size, serialized_data);
    }

    template <>
    void CSerializer::SerializeDynamicArray<std::string>(const char* data, std::string& serialized_data) const
    {
      auto sequence = reinterpret_cast<const rosidl_runtime_c__char__Sequence*>(data);

      SerializeSingle<array_size_t>(sequence->size, serialized_data);
      SerializeArray<std::string>(reinterpret_cast<const char*>(sequence->data), sequence->size, serialized_data);
    }

    template <>
    void CSerializer::SerializeDynamicArray<ros_message_t>(const char* data,
                                                           const rosidl_typesupport_introspection_c__MessageMember* member,
                                                           std::string& serialized_data) const
    {
      auto sequence = reinterpret_cast<const rosidl_runtime_c__char__Sequence*>(data);
      auto sequence_data = reinterpret_cast<const char*>(sequence->data);
      auto sub_members = GetMembers(member);

      SerializeSingle<array_size_t>(sequence->size, serialized_data);
      for (size_t i = 0; i < sequence->size; i++)
      {
        SerializeMessage(sequence_data, sub_members, serialized_data);
        sequence_data += sub_members->size_of_;
      }
    }

    template <typename T>
    void CSerializer::Serialize(const char* data,
                                const rosidl_typesupport_introspection_c__MessageMember& member,
                                std::string& serialized_data) const
    {
      if (member.is_array_)
      {
        //static array
        if (member.array_size_ > 0 && !member.is_upper_bound_)
        {
          SerializeArray<T>(data, member.array_size_, serialized_data);
        }
        else //dynamic array
        {
          SerializeDynamicArray<T>(data, serialized_data);
        }
      }
      else
      {
        SerializeSingle<T>(data, serialized_data);
      }
    }

    template <>
    void CSerializer::Serialize<ros_message_t>(const char* data,
                                               const rosidl_typesupport_introspection_c__MessageMember& member,
                                               std::string& serialized_data) const
    {
      if (member.is_array_)
      {
        if (member.array_size_ > 0 && !member.is_upper_bound_)
        {
          SerializeArray<ros_message_t>(data, &member, serialized_data);
        }
        else
        {
          SerializeDynamicArray<ros_message_t>(data, &member, serialized_data);
        }
      }
      else
      {
        auto sub_members = GetMembers(&member);
        SerializeMessage(data, sub_members, serialized_data);
      }
    }

    void CSerializer::SerializeMessage(const char* data,
                                       const rosidl_typesupport_introspection_c__MessageMembers* members,
                                       std::string& serialized_data) const
    {
      for (uint32_t i = 0; i < members->member_count_; i++)
      {
        auto member = members->members_ + i;
        auto member_data = data + member->offset_;

        switch (member->type_id_)
        {
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
          Serialize<std::string>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
          Serialize<bool>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_BYTE:
          Serialize<uint8_t>(member_data, *member, serialized_data); //-V1037
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
          Serialize<char>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
          Serialize<float>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
          Serialize<double>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
          Serialize<long double>(member_data, *member, serialized_data); //not documented mapping
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
          Serialize<int8_t>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
          Serialize<int16_t>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
          Serialize<int32_t>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
          Serialize<int64_t>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
          Serialize<uint8_t>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
          Serialize<uint16_t>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
          Serialize<uint32_t>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
          Serialize<uint64_t>(member_data, *member, serialized_data);
          break;
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
          Serialize<ros_message_t>(member_data, *member, serialized_data);
          break;
          //not documented
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
        case ::rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
          throw std::logic_error("Wide character/string serialization is unsupported.");
        }
      }
    }

    const std::string CSerializer::Serialize(const void* data)
    {
      std::string serialized_data;
      SerializeMessage(static_cast<const char*>(data), members_, serialized_data);
      return serialized_data;
    }

    const std::string CSerializer::GetMessageStringDescriptor() const
    {
      return "";
    }

  } // namespace rmw
} // namespace eCAL
