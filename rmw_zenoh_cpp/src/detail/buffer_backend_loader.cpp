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
#include <cstring>
#include <string>
#include <stdexcept>
#include <utility>

#include "rcutils/error_handling.h"
#include "rcutils/logging_macros.h"
#include "rosidl_buffer_backend_registry/buffer_backend_registry.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"

namespace rmw_zenoh_cpp
{

static const char * kLoggerName = "rmw_zenoh_cpp.buffer_backend_loader";

void initialize_buffer_backends(BufferBackendContext & context)
{
  context.registry = std::make_unique<rosidl_buffer_backend_registry::BufferBackendRegistry>();
  auto & registry = *context.registry;

  auto & backend_ops = context.serialization_context.descriptor_ops;
  auto & serializers = context.serialization_context.descriptor_serializers;
  auto backend_names = registry.get_backend_names();
  RCUTILS_LOG_DEBUG_NAMED(
    kLoggerName, "Buffer backends: found %zu backend(s)", backend_names.size());

  for (const auto & backend_name : backend_names) {
    auto backend = registry.create_backend_instance(backend_name);
    if (!backend) {
      RCUTILS_LOG_ERROR_NAMED(
        kLoggerName, "Backend '%s' pointer is null", backend_name.c_str());
      continue;
    }

    std::string backend_type = backend->get_backend_type();
    RCUTILS_LOG_DEBUG_NAMED(
      kLoggerName, "Processing backend '%s' (type: %s)",
      backend_name.c_str(), backend_type.c_str());

    context.backend_instances[backend_type] = backend;

    rosidl::BufferDescriptorOps ops;

    auto backend_ptr = backend;
    ops.create_descriptor_with_endpoint = [backend_ptr](
      const void * impl,
      const rmw_topic_endpoint_info_t & endpoint_info) -> std::shared_ptr<void> {
        return backend_ptr->create_descriptor_with_endpoint(impl, endpoint_info);
      };
    ops.from_descriptor_with_endpoint = [backend_ptr](
      const void * descriptor,
      const rmw_topic_endpoint_info_t & endpoint_info)
      -> std::unique_ptr<void, void (*)(void *)> {
        return backend_ptr->from_descriptor_with_endpoint(descriptor, endpoint_info);
      };

    backend_ops[backend_type] = ops;

    const rosidl_message_type_support_t * descriptor_ts =
      backend->get_descriptor_type_support();
    if (!descriptor_ts) {
      RCUTILS_LOG_ERROR_NAMED(
        kLoggerName,
        "  Backend '%s' returned null descriptor type support",
        backend_type.c_str());
      continue;
    }
    const rosidl_message_type_support_t * fastrtps_descriptor_ts = get_message_typesupport_handle(
      descriptor_ts, rosidl_typesupport_fastrtps_cpp::typesupport_identifier);
    if (!fastrtps_descriptor_ts) {
      RCUTILS_LOG_ERROR_NAMED(
        kLoggerName,
        "  Backend '%s' descriptor type support could not be resolved to "
        "rosidl_typesupport_fastrtps_cpp",
        backend_type.c_str());
      rcutils_reset_error();
      continue;
    }

    const auto * callbacks = static_cast<const message_type_support_callbacks_t *>(
      fastrtps_descriptor_ts->data);
    if (!callbacks) {
      RCUTILS_LOG_ERROR_NAMED(
        kLoggerName,
        "  Backend '%s' descriptor callbacks are null",
        backend_type.c_str());
      continue;
    }

    rosidl_typesupport_fastrtps_cpp::BufferDescriptorSerializers desc_ser;
    auto backend_ptr_for_desc = backend;
    desc_ser.serialize = [callbacks](
      eprosima::fastcdr::Cdr & cdr,
      const std::shared_ptr<void> & desc_ptr,
      const rmw_topic_endpoint_info_t & endpoint_info,
      const rosidl_typesupport_fastrtps_cpp::BufferSerializationContext & serialization_context)
      {
        if (!desc_ptr) {
          throw std::runtime_error("Descriptor pointer is null");
        }
        if (callbacks->cdr_serialize_with_endpoint) {
          callbacks->cdr_serialize_with_endpoint(
            desc_ptr.get(), cdr, endpoint_info, serialization_context);
        } else {
          callbacks->cdr_serialize(desc_ptr.get(), cdr);
        }
      };
    desc_ser.deserialize = [callbacks, backend_ptr_for_desc](
      eprosima::fastcdr::Cdr & cdr,
      const rmw_topic_endpoint_info_t & endpoint_info,
      const rosidl_typesupport_fastrtps_cpp::BufferSerializationContext & serialization_context)
      -> std::shared_ptr<void>
      {
        auto desc = backend_ptr_for_desc->create_empty_descriptor();
        if (!desc) {
          throw std::runtime_error("Backend returned null descriptor instance");
        }
        if (callbacks->cdr_deserialize_with_endpoint) {
          callbacks->cdr_deserialize_with_endpoint(
            cdr, desc.get(), endpoint_info, serialization_context);
        } else {
          callbacks->cdr_deserialize(cdr, desc.get());
        }
        return desc;
      };
    serializers[backend_type] = std::move(desc_ser);

    RCUTILS_LOG_DEBUG_NAMED(
      kLoggerName, "  FastCDR descriptor serializers registered for '%s'",
      backend_type.c_str());
  }
}

void shutdown_buffer_backends(BufferBackendContext & context)
{
  try {
    context.serialization_context.descriptor_ops.clear();
    context.serialization_context.descriptor_serializers.clear();
    context.backend_instances.clear();
    context.registry.reset();
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      kLoggerName, "Error during buffer backend shutdown: %s", e.what());
  }
}

}  // namespace rmw_zenoh_cpp
