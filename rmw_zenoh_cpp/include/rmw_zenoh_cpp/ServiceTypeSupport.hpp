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

#ifndef RMW_ZENOH_CPP__SERVICETYPESUPPORT_HPP_
#define RMW_ZENOH_CPP__SERVICETYPESUPPORT_HPP_

#include "rosidl_typesupport_zenoh_cpp/message_type_support.h"
#include "rosidl_typesupport_zenoh_cpp/service_type_support.h"
#include "TypeSupport.hpp"

namespace rmw_zenoh_cpp
{

class ServiceTypeSupport : public TypeSupport
{
protected:
  ServiceTypeSupport();
};

class RequestTypeSupport : public ServiceTypeSupport
{
public:
  explicit RequestTypeSupport(const service_type_support_callbacks_t * members);
};

class ResponseTypeSupport : public ServiceTypeSupport
{
public:
  explicit ResponseTypeSupport(const service_type_support_callbacks_t * members);
};

}  // namespace rmw_zenoh_cpp

#endif  // RMW_ZENOH_CPP__SERVICETYPESUPPORT_HPP_
