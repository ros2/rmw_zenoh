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
#include <memory>

#include <rosidl_typesupport_introspection_c/message_introspection.h>

#include "internal/typesupport/message_typesupport_base.hpp"
#include "internal/common.hpp"
#include "internal/serialization.hpp"

namespace eCAL
{
  namespace rmw
  {

    class CMessageTypeSupport : public MessageTypeSupport
    {
      const rosidl_message_type_support_t* type_support_;
      std::unique_ptr<Serializer> serializer_;
      std::unique_ptr<Deserializer> deserializer_;

      const rosidl_typesupport_introspection_c__MessageMembers* GetMembers() const
      {
        return GetCMembers(type_support_);
      }

    public:
      explicit CMessageTypeSupport(const rosidl_message_type_support_t* type_support)
        : type_support_(type_support),
          serializer_(CreateSerializer(GetMembers())),
          deserializer_(CreateDeserializer(GetMembers()))
      {
      }

      virtual const std::string GetMessageNamespace() const override
      {
        auto members = GetMembers();
        return ReplaceString(ReplaceString(members->message_namespace_, "::", "/"), "__", "/");
      }

      virtual const std::string GetMessageSimpleName() const override
      {
        auto members = GetMembers();
        return members->message_name_ ? members->message_name_ : "";
      }

      virtual const std::string GetMessageName() const override
      {
        return GetMessageNamespace() + "/" + GetMessageSimpleName();
      }

      virtual size_t GetTypeSize() const override
      {
        return GetMembers()->size_of_;
      }

      virtual const std::string Serialize(const void* data) override
      {
        return serializer_->Serialize(data);
      }

      virtual void Deserialize(void* message, const void* serialized_data, size_t size) override
      {
        deserializer_->Deserialize(message, serialized_data, size);
      }

      virtual std::string GetTypeDescriptor() const override
      {
        return serializer_->GetMessageStringDescriptor();
      }
    };

  } // namespace rmw
} // namespace eCAL
