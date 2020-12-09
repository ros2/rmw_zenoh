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

#include "serializer_cpp.hpp"

#include <string>
#include <vector>
#include <stdexcept>

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include "internal/common.hpp"

namespace eCAL
{
  namespace rmw
  {

    namespace ts_introspection = rosidl_typesupport_introspection_cpp;

    template <typename T>
    void CppSerializer::SerializeSingle(const char* data, std::string& serialized_data) const
    {
      serialized_data.insert(serialized_data.end(), data, data + sizeof(T));
    }

    template <typename T>
    void CppSerializer::SerializeSingle(const T& data, std::string& serialized_data) const
    {
      auto data_bytes = reinterpret_cast<const char*>(&data);

      SerializeSingle<T>(data_bytes, serialized_data);
    }

    template <>
    void CppSerializer::SerializeSingle<std::string>(const char* data, std::string& serialized_data) const
    {
      auto str = reinterpret_cast<const std::string*>(data);
      auto str_data = str->c_str();
      auto str_size = str->size();

      SerializeArraySize(*str, serialized_data);
      SerializeArray<char>(str_data, str_size, serialized_data);
    }

    template <typename ARR>
    void CppSerializer::SerializeArraySize(const ARR& array, std::string& serialized_data) const
    {
      array_size_t size = array.size();
      SerializeSingle<array_size_t>(size, serialized_data);
    }

    template <typename T>
    void CppSerializer::SerializeArray(const char* data, size_t count, std::string& serialized_data) const
    {
      serialized_data.insert(serialized_data.end(), data, data + count * sizeof(T));
      // TODO: PVS issue V1001 (https://www.viva64.com/en/w/v1001/print/)
      // what is this line supposed to do ?
      data += sizeof(T) * count;
    }

    template <>
    void CppSerializer::SerializeArray<std::string>(const char* data, size_t count, std::string& serialized_data) const
    {
      for (size_t i = 0; i < count; i++)
      {
        SerializeSingle<std::string>(data, serialized_data);
        data += sizeof(std::string);
      }
    }

    template <>
    void CppSerializer::SerializeArray<ros_message_t>(const char* data,
                                                      const ts_introspection::MessageMember* member,
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
    void CppSerializer::SerializeDynamicArray(const char* data, std::string& serialized_data) const
    {
      auto& array = *reinterpret_cast<const std::vector<T>*>(data);
      auto array_data = reinterpret_cast<const char*>(array.data());
      auto array_size = array.size();

      //reserve data for size and content of array to avoid multiple reallocations
      serialized_data.reserve(serialized_data.size() + array_size * sizeof(T) + sizeof(array_size_t));

      SerializeArraySize(array, serialized_data);
      SerializeArray<T>(array_data, array_size, serialized_data);
    }

    template <>
    void CppSerializer::SerializeDynamicArray<bool>(const char* data, std::string& serialized_data) const
    {
      auto& array = *reinterpret_cast<const std::vector<bool>*>(data);
      serialized_data.reserve(serialized_data.size() + array.size() * sizeof(bool) + sizeof(array_size_t));

      SerializeArraySize(array, serialized_data);
      serialized_data.insert(serialized_data.end(), array.begin(), array.end());
    }

    template <>
    void CppSerializer::SerializeDynamicArray<ros_message_t>(const char* data,
                                                             const ts_introspection::MessageMember* member,
                                                             std::string& serialized_data) const
    {
      auto vector = reinterpret_cast<const std::vector<char>*>(data);
      auto sub_members = GetMembers(member);
      auto vec_data = vector->data();
      array_size_t size = vector->size() / sub_members->size_of_;

      SerializeSingle(size, serialized_data);
      for (size_t i = 0; i < size; i++)
      {
        SerializeMessage(vec_data, sub_members, serialized_data);
        vec_data += sub_members->size_of_;
      }
    }

    template <typename T>
    void CppSerializer::Serialize(const char* data,
                                  const ts_introspection::MessageMember& member,
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
    void CppSerializer::Serialize<ros_message_t>(const char* data,
                                                 const ts_introspection::MessageMember& member,
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

    void CppSerializer::SerializeMessage(const char* data,
                                         const ts_introspection::MessageMembers* members,
                                         std::string& serialized_data) const
    {
      for (uint32_t i = 0; i < members->member_count_; i++)
      {
        const auto member = members->members_ + i;
        const auto member_data = data + member->offset_;

        switch (member->type_id_)
        {
        case ts_introspection::ROS_TYPE_STRING:
          Serialize<std::string>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_BOOLEAN:
          Serialize<bool>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_BYTE:
          Serialize<uint8_t>(member_data, *member, serialized_data); //-V1037
          break;
        case ts_introspection::ROS_TYPE_CHAR:
          Serialize<char>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_FLOAT:
          Serialize<float>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_DOUBLE:
          Serialize<double>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_LONG_DOUBLE:
          Serialize<long double>(member_data, *member, serialized_data); //not documented mapping
          break;
        case ts_introspection::ROS_TYPE_INT8:
          Serialize<int8_t>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_INT16:
          Serialize<int16_t>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_INT32:
          Serialize<int32_t>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_INT64:
          Serialize<int64_t>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_UINT8:
          Serialize<uint8_t>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_UINT16:
          Serialize<uint16_t>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_UINT32:
          Serialize<uint32_t>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_UINT64:
          Serialize<uint64_t>(member_data, *member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_MESSAGE:
          Serialize<ros_message_t>(member_data, *member, serialized_data);
          break;
          //not documented
        case ts_introspection::ROS_TYPE_WSTRING:
        case ts_introspection::ROS_TYPE_WCHAR:
          throw std::logic_error("Wide character/string serialization is unsupported.");
        }
      }
    }

    const std::string CppSerializer::Serialize(const void* data)
    {
      //it might be good idea to pre estimate and reserve data size in our payload vector
      std::string serialized_data;
      SerializeMessage(static_cast<const char*>(data), members_, serialized_data);
      return serialized_data;
    }

    const std::string CppSerializer::GetMessageStringDescriptor() const
    {
      return "";
    }

  } // namespace rmw
} // namespace eCAL
