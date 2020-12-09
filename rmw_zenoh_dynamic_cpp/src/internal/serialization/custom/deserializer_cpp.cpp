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

#include "deserializer_cpp.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include "internal/common.hpp"

namespace eCAL
{
  namespace rmw
  {

    namespace ts_introspection = rosidl_typesupport_introspection_cpp;

    template <typename T>
    void CppDeserializer::DeserializeSingle(char* member, const char** serialized_data)
    {
      std::copy_n(*serialized_data, sizeof(T), member);
      *serialized_data += sizeof(T);
    }

    template <>
    void CppDeserializer::DeserializeSingle<std::string>(char* member, const char** serialized_data)
    {
      auto str = reinterpret_cast<std::string*>(member);

      auto arr_size = DeserializeArraySize(serialized_data);

      if (arr_size > 0)
      {
        str->append(*serialized_data, arr_size);
        *serialized_data += arr_size;
      }
    }

    array_size_t CppDeserializer::DeserializeArraySize(const char** serialized_data)
    {
      auto arr_size = *reinterpret_cast<const array_size_t*>(*serialized_data);
      *serialized_data += sizeof(array_size_t);

      return arr_size;
    }

    template <typename T>
    void CppDeserializer::DeserializeArray(char* member, size_t size, const char** serialized_data)
    {
      std::copy_n(*serialized_data, sizeof(T) * size, member); //-V575
      *serialized_data += sizeof(T) * size;
    }

    template <>
    void CppDeserializer::DeserializeArray<std::string>(char* member, size_t size, const char** serialized_data)
    {
      for (size_t i = 0; i < size; i++)
      {
        DeserializeSingle<std::string>(member, serialized_data);
        member += sizeof(std::string);
      }
    }

    template <>
    void CppDeserializer::DeserializeArray<ros_message_t>(char* message,
      const ts_introspection::MessageMember* member,
      const char** serialized_data)
    {
      auto sub_members = GetMembers(member);
      for (size_t i = 0; i < member->array_size_; i++)
      {
        DeserializeMessage(serialized_data, sub_members, message);
        message += sub_members->size_of_;
      }
    }

    template <typename T>
    void CppDeserializer::DeserializeDynamicArray(char* member, const char** serialized_data)
    {
      auto array = reinterpret_cast<std::vector<T>*>(member);
      auto arr_size = DeserializeArraySize(serialized_data);
      array->resize(arr_size);
      auto arr_bytes = reinterpret_cast<char*>(array->data());
      DeserializeArray<T>(arr_bytes, arr_size, serialized_data);
    }

    template <>
    void CppDeserializer::DeserializeDynamicArray<bool>(char* member, const char** serialized_data)
    {
      auto array = reinterpret_cast<std::vector<bool>*>(member);
      auto arr_size = DeserializeArraySize(serialized_data);
      auto data = reinterpret_cast<const bool*>(*serialized_data);
      array->reserve(arr_size);
      for (array_size_t i = 0; i < arr_size; i++)
      {
        array->push_back(data[i]);
      }
      *serialized_data += arr_size;
    }

    template <>
    void CppDeserializer::DeserializeDynamicArray<ros_message_t>(char* message,
      const ts_introspection::MessageMember* member,
      const char** serialized_data)
    {
      auto vec = reinterpret_cast<std::vector<char>*>(message);
      auto arr_size = DeserializeArraySize(serialized_data);
      auto sub_members = GetMembers(member);

      vec->resize(arr_size * sub_members->size_of_);
      auto data = vec->data();
      for (array_size_t i = 0; i < arr_size; i++)
      {
        DeserializeMessage(serialized_data, sub_members, data);
        data += sub_members->size_of_;
      }
    }

    template <typename T>
    void CppDeserializer::Deserialize(char* member_data,
      const ts_introspection::MessageMember* member,
      const char** serialized_data)
    {
      if (member->is_array_)
      {
        //static array
        if (member->array_size_ > 0 && !member->is_upper_bound_)
        {
          DeserializeArray<T>(member_data, member->array_size_, serialized_data);
        }
        else //dynamic array
        {
          DeserializeDynamicArray<T>(member_data, serialized_data);
        }
      }
      else
      {
        DeserializeSingle<T>(member_data, serialized_data);
      }
    }

    template <>
    void CppDeserializer::Deserialize<ros_message_t>(char* member_data,
      const ts_introspection::MessageMember* member,
      const char** serialized_data)
    {
      if (member->is_array_)
      {
        if (member->array_size_ > 0 && !member->is_upper_bound_)
        {
          DeserializeArray<ros_message_t>(member_data, member, serialized_data);
        }
        else //dynamic array
        {
          DeserializeDynamicArray<ros_message_t>(member_data, member, serialized_data);
        }
      }
      else
      {
        auto members = GetMembers(member);
        DeserializeMessage(serialized_data, members, member_data);
      }
    }

    void CppDeserializer::DeserializeMessage(const char** serialized_data,
      const ts_introspection::MessageMembers* members,
      char* message)
    {
      for (size_t i = 0; i < members->member_count_; i++)
      {
        const auto member = members->members_ + i;
        auto member_data = message + member->offset_;

        switch (member->type_id_)
        {
        case ts_introspection::ROS_TYPE_STRING:
          Deserialize<std::string>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_BOOLEAN:
          Deserialize<bool>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_BYTE:
          Deserialize<uint8_t>(member_data, member, serialized_data); //-V1037
          break;
        case ts_introspection::ROS_TYPE_CHAR:
          Deserialize<char>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_FLOAT:
          Deserialize<float>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_DOUBLE:
          Deserialize<double>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_LONG_DOUBLE:
          Deserialize<long double>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_INT8:
          Deserialize<int8_t>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_INT16:
          Deserialize<int16_t>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_INT32:
          Deserialize<int32_t>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_INT64:
          Deserialize<int64_t>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_UINT8:
          Deserialize<uint8_t>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_UINT16:
          Deserialize<uint16_t>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_UINT32:
          Deserialize<uint32_t>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_UINT64:
          Deserialize<uint64_t>(member_data, member, serialized_data);
          break;
        case ts_introspection::ROS_TYPE_MESSAGE:
          Deserialize<ros_message_t>(member_data, member, serialized_data);
          break;
          //these 2 aren't documented
        case ts_introspection::ROS_TYPE_WCHAR:
        case ts_introspection::ROS_TYPE_WSTRING:
          throw std::logic_error("Wide character/string deserialization is unsupported.");
        }
      }
    }

    void CppDeserializer::Deserialize(void* message, const void* serialized_data, size_t /* size */)
    {
      auto serialized_bytes = static_cast<const char*>(serialized_data);
      auto message_bytes = static_cast<char*>(message);
      DeserializeMessage(&serialized_bytes, members_, message_bytes);
    }

  } // namespace rmw
} // namespace eCAL
