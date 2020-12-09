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

namespace eCAL
{
  namespace rmw
  {

    class ServiceTypeSupport
    {
    public:
      virtual ~ServiceTypeSupport() = default;
      virtual const std::string GetServiceNamespace() const = 0;
      virtual const std::string GetServiceSimpleName() const = 0;
      virtual const std::string GetServiceName() const = 0;
      virtual const std::string GetRequestMessageNamespace() const = 0;
      virtual const std::string GetRequestMessageName() const = 0;
      virtual const std::string GetResponseMessageNamespace() const = 0;
      virtual const std::string GetResponseMessageName() const = 0;
      virtual const std::string SerializeRequest(const void* data) = 0;
      virtual const std::string SerializeResponse(const void* data) = 0;
      virtual void DeserializeRequest(void* message, const void* serialized_data, size_t size) = 0;
      virtual void DeserializeResponse(void* message, const void* serialized_data, size_t size) = 0;
    };

  } // namespace rmw
} // namespace eCAL
