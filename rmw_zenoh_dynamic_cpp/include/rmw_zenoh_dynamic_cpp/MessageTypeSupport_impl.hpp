// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef RMW_ZENOH_DYNAMIC_CPP__MESSAGETYPESUPPORT_IMPL_HPP_
#define RMW_ZENOH_DYNAMIC_CPP__MESSAGETYPESUPPORT_IMPL_HPP_

#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

#include <cassert>
#include <memory>
#include <sstream>
#include <string>

#include "rcpputils/find_and_replace.hpp"

#include "rmw_zenoh_dynamic_cpp/MessageTypeSupport.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace rmw_zenoh_dynamic_cpp
{

template<typename MembersType>
MessageTypeSupport<MembersType>::MessageTypeSupport(
  const MembersType * members, const void * ros_type_support)
: TypeSupport<MembersType>(ros_type_support)
{
  assert(members);
  this->members_ = members;

  std::ostringstream ss;
  std::string message_namespace(this->members_->message_namespace_);
  std::string message_name(this->members_->message_name_);
  if (!message_namespace.empty()) {
    // Find and replace C namespace separator with C++, in case this is using C typesupport
    message_namespace = rcpputils::find_and_replace(message_namespace, "__", "::");
    ss << message_namespace << "::";
  }
  ss << "dds_::" << message_name << "_";
  this->setName(ss.str().c_str());

  // Fully bound by default
  this->max_size_bound_ = true;
  // Encapsulation size
  this->m_typeSize = 4;
  if (this->members_->member_count_ != 0) {
    this->m_typeSize += static_cast<uint32_t>(this->calculateMaxSerializedSize(members, 0));
  } else {
    this->m_typeSize++;
  }
}

}  // namespace rmw_zenoh_dynamic_cpp

#endif  // RMW_ZENOH_DYNAMIC_CPP__MESSAGETYPESUPPORT_IMPL_HPP_
