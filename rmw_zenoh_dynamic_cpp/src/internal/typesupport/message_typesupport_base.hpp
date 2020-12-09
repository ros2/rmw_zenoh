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

    class MessageTypeSupport
    {
    public:
      virtual ~MessageTypeSupport() = default;
      virtual const std::string GetMessageNamespace() const = 0;
      virtual const std::string GetMessageSimpleName() const = 0;
      virtual const std::string GetMessageName() const = 0;
      virtual size_t GetTypeSize() const = 0;
      virtual const std::string Serialize(const void* data) = 0;
      virtual void Deserialize(void* message, const void* serialized_data, size_t size) = 0;
      virtual std::string GetTypeDescriptor() const = 0;
    };

  } // namespace rmw
} // namespace eCAL
