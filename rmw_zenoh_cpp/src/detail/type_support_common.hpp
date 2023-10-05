// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Copyright 2023 Open Source Robotics Foundation, Inc.
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

// This file is originally from:
// https://github.com/ros2/rmw_fastrtps/blob/469624e3d483290d6f88fe4b89ee5feaa7694e61/rmw_fastrtps_cpp/src/type_support_common.hpp

#ifndef DETAIL__TYPE_SUPPORT_COMMON_HPP_
#define DETAIL__TYPE_SUPPORT_COMMON_HPP_

#include <sstream>
#include <string>

#include "rmw/error_handling.h"

#include "type_support.hpp"
#include "message_type_support.hpp"
#include "service_type_support.hpp"

#include "identifier.hpp"

#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#define RMW_FASTRTPS_CPP_TYPESUPPORT_C rosidl_typesupport_fastrtps_c__identifier
#define RMW_FASTRTPS_CPP_TYPESUPPORT_CPP rosidl_typesupport_fastrtps_cpp::typesupport_identifier

#define RMW_ZENOH_CPP_TYPESUPPORT_C rosidl_typesupport_fastrtps_c__identifier
#define RMW_ZENOH_CPP_TYPESUPPORT_CPP rosidl_typesupport_fastrtps_cpp::typesupport_identifier

inline std::string
_create_type_name(
  std::string message_namespace,
  std::string message_name)
{
  std::ostringstream ss;
  if (!message_namespace.empty()) {
    ss << message_namespace << "::";
  }
  ss << "dds_::" << message_name << "_";
  return ss.str();
}

inline std::string
_create_type_name(
  const message_type_support_callbacks_t * members)
{
  if (!members) {
    RMW_SET_ERROR_MSG("members handle is null");
    return "";
  }
  std::string message_namespace(members->message_namespace_);
  std::string message_name(members->message_name_);
  return _create_type_name(message_namespace, message_name);
}

#endif  // DETAIL__TYPE_SUPPORT_COMMON_HPP_
