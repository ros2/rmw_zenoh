// Copyright 2020 ADLINK, Inc.
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

#ifndef RMW_ZENOH_DYNAMIC_CPP__SERVICETYPESUPPORT_IMPL_HPP_
#define RMW_ZENOH_DYNAMIC_CPP__SERVICETYPESUPPORT_IMPL_HPP_

#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>
#include <cassert>
#include <sstream>
#include <string>

#include "rcpputils/find_and_replace.hpp"

#include "rmw_zenoh_dynamic_cpp/ServiceTypeSupport.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace rmw_zenoh_dynamic_cpp
{

template<typename ServiceMembersType, typename MessageMembersType>
RequestTypeSupport<ServiceMembersType, MessageMembersType>::RequestTypeSupport(
  const ServiceMembersType * members, const void * ros_type_support)
: TypeSupport<MessageMembersType>(ros_type_support)
{
  assert(members);
  this->members_ = members->request_members_;

  std::ostringstream ss;
  std::string service_namespace(members->service_namespace_);
  std::string service_name(members->service_name_);
  if (!service_namespace.empty()) {
    // Find and replace C namespace separator with C++, in case this is using C typesupport
    service_namespace = rcpputils::find_and_replace(service_namespace, "__", "::");
    ss << service_namespace << "::";
  }
  ss << "dds_::" << service_name << "_Request_";
  this->setName(ss.str().c_str());

  // Fully bound by default
  this->max_size_bound_ = true;
  // Encapsulation size
  this->m_typeSize = 4;
  if (this->members_->member_count_ != 0) {
    this->m_typeSize += static_cast<uint32_t>(this->calculateMaxSerializedSize(this->members_, 0));
  } else {
    this->m_typeSize++;
  }
}

template<typename ServiceMembersType, typename MessageMembersType>
ResponseTypeSupport<ServiceMembersType, MessageMembersType>::ResponseTypeSupport(
  const ServiceMembersType * members, const void * ros_type_support)
: TypeSupport<MessageMembersType>(ros_type_support)
{
  assert(members);
  this->members_ = members->response_members_;

  std::ostringstream ss;
  std::string service_namespace(members->service_namespace_);
  std::string service_name(members->service_name_);
  if (!service_namespace.empty()) {
    // Find and replace C namespace separator with C++, in case this is using C typesupport
    service_namespace = rcpputils::find_and_replace(service_namespace, "__", "::");
    ss << service_namespace << "::";
  }
  ss << "dds_::" << service_name << "_Response_";
  this->setName(ss.str().c_str());

  // Fully bound by default
  this->max_size_bound_ = true;
  // Encapsulation size
  this->m_typeSize = 4;
  if (this->members_->member_count_ != 0) {
    this->m_typeSize += static_cast<uint32_t>(this->calculateMaxSerializedSize(this->members_, 0));
  } else {
    this->m_typeSize++;
  }
}

}  // namespace rmw_zenoh_dynamic_cpp

#endif  // RMW_ZENOH_DYNAMIC_CPP__SERVICETYPESUPPORT_IMPL_HPP_
