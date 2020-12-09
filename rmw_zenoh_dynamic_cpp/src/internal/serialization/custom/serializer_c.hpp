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

#pragma once

#include <string>

#include "rosidl_typesupport_introspection_c/message_introspection.h"

#include "internal/serialization/serializer.hpp"

namespace eCAL
{
  namespace rmw
  {

    class CSerializer : public Serializer
    {
      const rosidl_typesupport_introspection_c__MessageMembers* members_;

      template <typename T>
      void SerializeSingle(const char* data, std::string& serialized_data) const;

      template <typename T>
      void SerializeSingle(const T& data, std::string& serialized_data) const;

      template <typename T>
      void SerializeArray(const char* data, size_t count, std::string& serialized_data) const;

      template <typename T>
      void SerializeDynamicArray(const char* data, std::string& serialized_data) const;

      template <typename T>
      void Serialize(const char* data,
                     const rosidl_typesupport_introspection_c__MessageMember& member,
                     std::string& serialized_data) const;

      template <typename T>
      void SerializeArray(const char* data,
                          const rosidl_typesupport_introspection_c__MessageMember* member,
                          std::string& serialized_data) const;

      template <typename T>
      void SerializeDynamicArray(const char* data,
                                 const rosidl_typesupport_introspection_c__MessageMember* member,
                                 std::string& serialized_data) const;

      void SerializeMessage(const char* data,
                            const rosidl_typesupport_introspection_c__MessageMembers* members,
                            std::string& serialized_data) const;

    public:
      explicit CSerializer(const rosidl_typesupport_introspection_c__MessageMembers* members)
        : members_(members)
      {
      }

      virtual const std::string Serialize(const void* data) override;
      virtual const std::string GetMessageStringDescriptor() const override;
    };

  } // namespace rmw
} // namespace eCAL
