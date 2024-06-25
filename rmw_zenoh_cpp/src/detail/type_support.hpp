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
// https://github.com/ros2/rmw_fastrtps/blob/b13e134cea2852aba210299bef6f4df172d9a0e3/rmw_fastrtps_cpp/include/rmw_fastrtps_cpp/TypeSupport.hpp

#ifndef DETAIL__TYPE_SUPPORT_HPP_
#define DETAIL__TYPE_SUPPORT_HPP_

#include <string>

#include "fastcdr/Cdr.h"

#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

namespace rmw_zenoh_cpp
{
///=============================================================================
enum SerializedDataType
{
  FASTRTPS_SERIALIZED_DATA_TYPE_CDR_BUFFER,
  FASTRTPS_SERIALIZED_DATA_TYPE_DYNAMIC_MESSAGE,
  FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE
};

///=============================================================================
// Publishers write method will receive a pointer to this struct
struct SerializedData
{
  SerializedDataType type;  // The type of the next field
  void * data;
  const void * impl;  // RMW implementation specific data
};

///=============================================================================
class TypeSupport
{
public:
  void set_name(const char * name);

  const char * get_name() const;

  size_t get_estimated_serialized_size(const void * ros_message, const void * impl) const;

  bool serialize_ros_message(
    const void * ros_message, eprosima::fastcdr::Cdr & ser, const void * impl) const;

  bool deserialize_ros_message(
    eprosima::fastcdr::Cdr & deser, void * ros_message, const void * impl) const;

  virtual ~TypeSupport() {}

protected:
  TypeSupport();

  bool max_size_bound_;
  bool is_plain_;

  void set_members(const message_type_support_callbacks_t * members);

private:
  const message_type_support_callbacks_t * members_;
  bool has_data_;

  uint32_t type_size_;

  std::string topic_data_type_name_;
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__TYPE_SUPPORT_HPP_
