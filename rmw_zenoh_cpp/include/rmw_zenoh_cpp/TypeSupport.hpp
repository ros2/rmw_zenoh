#ifndef RMW_ZENOH_CPP__TYPESUPPORT_HPP_
#define RMW_ZENOH_CPP__TYPESUPPORT_HPP_

// #include <fastrtps/TopicDataType.h>

#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>
#include <cassert>
#include <string>

#include "rosidl_typesupport_zenoh_cpp/message_type_support.h"

namespace rmw_zenoh_cpp
{
// Used with rmw_publish
struct SerializedData
{
  void * data;  // The stored, serialized data
  const void * impl;  // Type support callback struct pointer

  // CDR
  // size_t 	encapsulation;
  size_t 	length;
  size_t 	max_size;
  // size_t 	pos;
};

class TypeSupport
{
public:
  size_t getEstimatedSerializedSize(const void * ros_message);

  bool serializeROSmessage(const void * ros_message,
                           eprosima::fastcdr::Cdr & ser,
                           const void * impl) const;

  bool deserializeROSmessage(eprosima::fastcdr::Cdr & deser,
                             void * ros_message,
                             const void * impl) const;

protected:
  TypeSupport();

  void set_members(const message_type_support_callbacks_t * members);

private:
  const message_type_support_callbacks_t * members_;
  bool has_data_;
  bool max_size_bound_;

  size_t type_size_;
};

}  // namespace rmw_zenoh_cpp

#endif  // RMW_ZENOH_CPP__TYPESUPPORT_HPP_
