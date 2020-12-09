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

#include <rmw/get_topic_names_and_types.h>

#include <rmw/rmw.h>

#include <tuple>
#include <algorithm>

#include "internal/common.hpp"
#include "internal/qos.hpp"
#include "internal/graph.hpp"

rmw_ret_t rmw_get_topic_names_and_types(const rmw_node_t* node,
                                        rcutils_allocator_t* allocator,
                                        bool no_demangle,
                                        rmw_names_and_types_t* topic_names_and_types)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_names_and_types, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto topics = eCAL::rmw::Graph::GetTopics();
  auto init_success = rmw_names_and_types_init(topic_names_and_types, topics.size(), allocator);
  if (init_success != RMW_RET_OK)
  {
    RMW_SET_ERROR_MSG("Failed to initialize service_names_and_types.");
    return init_success;
  }

  try
  {
    std::transform(topics.begin(), topics.end(), eCAL::rmw::RosArray::Begin(*topic_names_and_types),
      [no_demangle, allocator](auto& topic) {
        std::string demangled_name{ no_demangle ? topic.name : eCAL::rmw::DemangleTopicName(topic.name) };
        auto name = eCAL::rmw::ConstructCString(demangled_name);

        rcutils_string_array_t types;
        auto init_success = rcutils_string_array_init(&types, 1, allocator);
        if (init_success != RMW_RET_OK)
        {
          throw std::runtime_error("Failed to init types.");
        }
        types.data[0] = eCAL::rmw::ConstructCString(topic.type);

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
