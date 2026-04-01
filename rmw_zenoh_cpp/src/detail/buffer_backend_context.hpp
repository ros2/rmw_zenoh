// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef DETAIL__BUFFER_BACKEND_CONTEXT_HPP_
#define DETAIL__BUFFER_BACKEND_CONTEXT_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rosidl_buffer_backend/buffer_backend.hpp"
#include "rosidl_buffer_backend_registry/buffer_backend_registry.hpp"
#include "rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp"

namespace rmw_zenoh_cpp
{

/// Per-rmw_context bundle of buffer backend state.
struct BufferBackendContext
{
  rosidl_typesupport_fastrtps_cpp::BufferSerializationContext serialization_context;
  std::unique_ptr<rosidl_buffer_backend_registry::BufferBackendRegistry> registry;
  std::unordered_map<std::string, std::shared_ptr<rosidl::BufferBackend>> backend_instances;
};

}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__BUFFER_BACKEND_CONTEXT_HPP_
