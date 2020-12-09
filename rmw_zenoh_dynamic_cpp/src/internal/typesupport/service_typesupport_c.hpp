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

#include <rosidl_typesupport_introspection_c/service_introspection.h>

#include "internal/typesupport/service_typesupport_base.hpp"
#include "internal/common.hpp"
#include "internal/serialization.hpp"

namespace eCAL
{
  namespace rmw
  {

    class CServiceTypeSupport : public ServiceTypeSupport
    {
      const rosidl_service_type_support_t* type_support_;
      std::unique_ptr<Serializer> request_serializer_;
      std::unique_ptr<Serializer> response_serializer_;
      std::unique_ptr<Deserializer> request_deserializer_;
      std::unique_ptr<Deserializer> response_deserializer_;

      const rosidl_typesupport_introspection_c__ServiceMembers* GetMembers() const
      {
        return GetCMembers(type_support_);
      }

    public:
      explicit CServiceTypeSupport(const rosidl_service_type_support_t* type_support)
        : type_support_(type_support),
          request_serializer_(CreateSerializer(GetMembers()->request_members_)),
          response_serializer_(CreateSerializer(GetMembers()->response_members_)),
          request_deserializer_(CreateDeserializer(GetMembers()->request_members_)),
          response_deserializer_(CreateDeserializer(GetMembers()->response_members_))
      {
      }

      virtual const std::string GetServiceNamespace() const override
      {
        return GetMembers()->service_namespace_;
      }

      virtual const std::string GetServiceSimpleName() const override
      {
        return GetMembers()->service_name_;
      }

      virtual const std::string GetServiceName() const override
      {
        return GetServiceNamespace() + "::" + GetServiceSimpleName();
      }

      virtual const std::string GetRequestMessageNamespace() const override
      {
        return GetMembers()->request_members_->message_namespace_;
      }

      virtual const std::string GetRequestMessageName() const override
      {
        return GetMembers()->request_members_->message_name_;
      }

      virtual const std::string GetResponseMessageNamespace() const override
      {
        return GetMembers()->response_members_->message_namespace_;
      }

      virtual const std::string GetResponseMessageName() const override
      {
        return GetMembers()->response_members_->message_name_;
      }

      virtual const std::string SerializeRequest(const void* data) override
      {
        return request_serializer_->Serialize(data);
      }

      virtual const std::string SerializeResponse(const void* data) override
      {
        return response_serializer_->Serialize(data);
      }

      virtual void DeserializeRequest(void* message, const void* serialized_data, size_t size) override
      {
        request_deserializer_->Deserialize(message, serialized_data, size);
      }

      virtual void DeserializeResponse(void* message, const void* serialized_data, size_t size) override
      {
        response_deserializer_->Deserialize(message, serialized_data, size);
      }
    };

  } // namespace rmw
} // namespace eCAL
