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

#ifndef DETAIL__BUFFER_BACKEND_LOADER_HPP_
#define DETAIL__BUFFER_BACKEND_LOADER_HPP_

namespace rmw_zenoh_cpp
{

/// Load buffer backend plugins and register them with FastCDR serialization.
/// Populates the global BackendDescriptorOps and DescriptorSerializers maps
/// in rosidl_typesupport_fastrtps_cpp so that buffer-aware message types can
/// serialize/deserialize vendor-specific descriptors.
void initialize_buffer_backends();

/// Clear global serialization maps to release plugin references before unloading.
/// Must be called before BufferBackendRegistry singleton is destroyed to prevent
/// ClassLoader from trying to unload while objects still exist.
void shutdown_buffer_backends();

}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__BUFFER_BACKEND_LOADER_HPP_
