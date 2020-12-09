// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef IMPL__TYPE_SUPPORT_COMMON_HPP_
#define IMPL__TYPE_SUPPORT_COMMON_HPP_

#include <sstream>
#include <string>

#include "rmw/error_handling.h"

#include "rmw_zenoh_common_cpp/TypeSupport.hpp"
#include "rmw_zenoh_common_cpp/MessageTypeSupport.hpp"
#include "rmw_zenoh_common_cpp/ServiceTypeSupport.hpp"

#include "rosidl_typesupport_zenoh_c/identifier.h"
#include "rosidl_typesupport_zenoh_cpp/identifier.hpp"
#include "rosidl_typesupport_zenoh_cpp/message_type_support.h"
#include "rosidl_typesupport_zenoh_cpp/service_type_support.h"

#define RMW_ZENOH_CPP_TYPESUPPORT_C rosidl_typesupport_zenoh_c__identifier
#define RMW_ZENOH_CPP_TYPESUPPORT_CPP rosidl_typesupport_zenoh_cpp::typesupport_identifier

#endif  // IMPL__TYPE_SUPPORT_COMMON_HPP_
