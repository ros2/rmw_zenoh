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

#include "buffer_backend_loader.hpp"

#include <memory>
#include <string>

#include "rcutils/logging_macros.h"
#include "rosidl_buffer_backend_registry/buffer_backend_registry.hpp"
#include "rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp"

namespace rmw_zenoh_cpp
{

static const char * kLoggerName = "rmw_zenoh_cpp.buffer_backend_loader";

void initialize_buffer_backends()
{
  auto & registry = rosidl_buffer_backend_registry::BufferBackendRegistry::get_instance();
  registry.load_plugins();

  auto & backend_ops = rosidl_typesupport_fastrtps_cpp::get_backend_descriptor_ops();
  auto backend_names = registry.get_backend_names();
  RCUTILS_LOG_INFO_NAMED(
    kLoggerName, "Buffer backends: found %zu backend(s)", backend_names.size());

  for (const auto & backend_name : backend_names) {
    auto backend = registry.get_backend(backend_name);
    if (!backend) {
      RCUTILS_LOG_ERROR_NAMED(
        kLoggerName, "Backend '%s' pointer is null", backend_name.c_str());
      continue;
    }

    std::string backend_type = backend->get_backend_type();
    RCUTILS_LOG_INFO_NAMED(
      kLoggerName, "Processing backend '%s' (type: %s)",
      backend_name.c_str(), backend_type.c_str());

    rosidl_typesupport_fastrtps_cpp::BufferDescriptorOps ops;

    auto backend_ptr = backend;
    ops.create_descriptor_with_endpoint = [backend_ptr](
      const std::shared_ptr<void> & impl,
      const rmw_topic_endpoint_info_t & endpoint_info) -> std::shared_ptr<void> {
        return backend_ptr->create_descriptor_with_endpoint(impl, endpoint_info);
      };
    ops.from_descriptor_with_endpoint = [backend_ptr](
      const std::shared_ptr<void> & descriptor,
      const rmw_topic_endpoint_info_t & endpoint_info) -> std::shared_ptr<void> {
        return backend_ptr->from_descriptor_with_endpoint(descriptor, endpoint_info);
      };

    backend_ops[backend_type] = ops;

    auto & serializers = rosidl_typesupport_fastrtps_cpp::get_descriptor_serializers();
    if (serializers.find(backend_type) != serializers.end()) {
      RCUTILS_LOG_INFO_NAMED(
        kLoggerName, "  FastCDR descriptor serializers registered for '%s'",
        backend_type.c_str());
    } else {
      RCUTILS_LOG_ERROR_NAMED(
        kLoggerName,
        "  Backend '%s' did not register FastCDR descriptor serializers. "
        "Ensure the backend constructor calls "
        "rosidl_typesupport_fastrtps_cpp::register_buffer_descriptor<DescriptorMsgT>()",
        backend_type.c_str());
    }
  }
}

void shutdown_buffer_backends()
{
  try {
    auto & backend_ops = rosidl_typesupport_fastrtps_cpp::get_backend_descriptor_ops();
    backend_ops.clear();

    auto & serializers = rosidl_typesupport_fastrtps_cpp::get_descriptor_serializers();
    serializers.clear();
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      kLoggerName, "Warning during buffer backend shutdown: %s", e.what());
  }

  try {
    rosidl_buffer_backend_registry::BufferBackendRegistry::get_instance().clear_global_state();
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      kLoggerName, "Warning clearing backend registry: %s", e.what());
  }
}

}  // namespace rmw_zenoh_cpp
