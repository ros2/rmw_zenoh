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

#include <rmw/get_node_info_and_types.h>

#include <rmw/rmw.h>

#include "internal/common.hpp"
#include "internal/graph.hpp"

rmw_ret_t rmw_get_subscriber_names_and_types_by_node(const rmw_node_t* node,
                                                     rcutils_allocator_t* allocator,
                                                     const char* node_name,
                                                     const char* node_namespace,
                                                     bool no_demangle,
                                                     rmw_names_and_types_t* topic_names_and_types)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_names_and_types, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto subs = eCAL::rmw::Graph::GetSubscribers(node_namespace, node_name);

  auto init_success = rmw_names_and_types_init(topic_names_and_types, subs.subscribers().size(), allocator);
  if (init_success != RMW_RET_OK)
  {
    RMW_SET_ERROR_MSG("Failed to initialize topic_names_and_types.");
    return init_success;
  }

  try
  {
    std::transform(subs.subscribers().begin(), subs.subscribers().end(),
      eCAL::rmw::RosArray::Begin(*topic_names_and_types),
      [&allocator, no_demangle](auto& sub) {
        auto demangled_name{ no_demangle ? sub.name() : eCAL::rmw::DemangleTopicName(sub.name()) };
        auto name = eCAL::rmw::ConstructCString(demangled_name);

        rcutils_string_array_t types;
        auto init_success = rcutils_string_array_init(&types, 1, allocator);
        if (init_success != RMW_RET_OK)
        {
          throw std::runtime_error("Failed to init types.");
        }
        types.data[0] = eCAL::rmw::ConstructCString(sub.type());

        return std::make_tuple(name, types);
      });
  }
  catch (const std::runtime_error& e)
  {
    RMW_SET_ERROR_MSG(e.what());
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

rmw_ret_t rmw_get_publisher_names_and_types_by_node(const rmw_node_t* node,
  rcutils_allocator_t* allocator,
  const char* node_name,
  const char* node_namespace,
  bool no_demangle,
  rmw_names_and_types_t* topic_names_and_types)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_names_and_types, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto pubs = eCAL::rmw::Graph::GetPublishers(node_namespace, node_name);

  auto init_success = rmw_names_and_types_init(topic_names_and_types, pubs.publishers().size(), allocator);
  if (init_success != RMW_RET_OK)
  {
    RMW_SET_ERROR_MSG("Failed to initialize topic_names_and_types.");
    return init_success;
  }

  try
  {
    std::transform(pubs.publishers().begin(), pubs.publishers().end(),
      eCAL::rmw::RosArray::Begin(*topic_names_and_types),
      [&allocator, no_demangle](auto& pub) {
        auto demangled_name{ no_demangle ? pub.name() : eCAL::rmw::DemangleTopicName(pub.name()) };
        auto name = eCAL::rmw::ConstructCString(demangled_name);

        rcutils_string_array_t types;
        auto init_success = rcutils_string_array_init(&types, 1, allocator);
        if (init_success != RMW_RET_OK)
        {
          throw std::runtime_error("Failed to init types.");
        }
        types.data[0] = eCAL::rmw::ConstructCString(pub.type());

        return std::make_tuple(name, types);
      });
  }
  catch (const std::runtime_error& e)
  {
    RMW_SET_ERROR_MSG(e.what());
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t rmw_get_service_names_and_types_by_node(const rmw_node_t* node,
  rcutils_allocator_t* allocator,
  const char* node_name,
  const char* node_namespace,
  rmw_names_and_types_t* service_names_and_types)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_names_and_types, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto services = eCAL::rmw::Graph::GetServices(node_namespace, node_name);

  auto init_success = rmw_names_and_types_init(service_names_and_types, services.services().size(), allocator);
  if (init_success != RMW_RET_OK)
  {
    RMW_SET_ERROR_MSG("Failed to initialize service_names_and_types.");
    return init_success;
  }

  try
  {
    std::transform(services.services().begin(), services.services().end(),
      eCAL::rmw::RosArray::Begin(*service_names_and_types),
      [&allocator](auto& ser) {
        auto demangled_name = eCAL::rmw::DemangleServiceName(ser.name());
        auto name = eCAL::rmw::ConstructCString(demangled_name);

        rcutils_string_array_t types;
        auto init_success = rcutils_string_array_init(&types, 2, allocator);
        if (init_success != RMW_RET_OK)
        {
          throw std::runtime_error("Failed to init types.");
        }
        types.data[0] = eCAL::rmw::ConstructCString(ser.request_type());
        types.data[1] = eCAL::rmw::ConstructCString(ser.response_type());

        return std::make_tuple(name, types);
      });
  }
  catch (const std::runtime_error& e)
  {
    RMW_SET_ERROR_MSG(e.what());
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t rmw_get_client_names_and_types_by_node(const rmw_node_t* node,
                                                 rcutils_allocator_t* allocator,
                                                 const char* node_name,
                                                 const char* node_namespace,
                                                 rmw_names_and_types_t* service_names_and_types)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_names_and_types, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto clients = eCAL::rmw::Graph::GetClients(node_namespace, node_name);

  auto init_success = rmw_names_and_types_init(service_names_and_types, clients.clients().size(), allocator);
  if (init_success != RMW_RET_OK)
  {
    RMW_SET_ERROR_MSG("Failed to initialize service_names_and_types.");
    return init_success;
  }

  try
  {
    std::transform(clients.clients().begin(), clients.clients().end(),
      eCAL::rmw::RosArray::Begin(*service_names_and_types),
      [&allocator](auto& cli) {
        auto demangled_name = eCAL::rmw::DemangleServiceName(cli.name());
        auto name = eCAL::rmw::ConstructCString(demangled_name);

        rcutils_string_array_t types;
        auto init_success = rcutils_string_array_init(&types, 2, allocator);
        if (init_success != RMW_RET_OK)
        {
          throw std::runtime_error("Failed to init types.");
        }
        types.data[0] = eCAL::rmw::ConstructCString(cli.request_type());
        types.data[1] = eCAL::rmw::ConstructCString(cli.response_type());

        return std::make_tuple(name, types);
      });
  }
  catch (const std::runtime_error& e)
  {
    RMW_SET_ERROR_MSG(e.what());
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}
