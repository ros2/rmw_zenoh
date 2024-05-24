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

// Parts of this file are originally from:
// https://github.com/ros2/rmw_fastrtps/blob/469624e3d483290d6f88fe4b89ee5feaa7694e61/rmw_fastrtps_cpp/src/type_support_common.hpp

#include <string>

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"

#include "service_type_support.hpp"
#include "type_support_common.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
ServiceTypeSupport::ServiceTypeSupport()
{
}

///=============================================================================
RequestTypeSupport::RequestTypeSupport(const service_type_support_callbacks_t * members)
{
  assert(members);

  auto msg = static_cast<const message_type_support_callbacks_t *>(
    members->request_members_->data);
  std::string name = _create_type_name(msg);  // + "Request_";
  this->set_name(name.c_str());

  set_members(msg);
}

///=============================================================================
ResponseTypeSupport::ResponseTypeSupport(const service_type_support_callbacks_t * members)
{
  assert(members);

  auto msg = static_cast<const message_type_support_callbacks_t *>(
    members->response_members_->data);
  std::string name = _create_type_name(msg);  // + "Response_";
  this->set_name(name.c_str());

  set_members(msg);
}
}  // namespace rmw_zenoh_cpp
