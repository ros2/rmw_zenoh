#ifndef RMW_ZENOH_CPP__MESSAGETYPESUPPORT_HPP_
#define RMW_ZENOH_CPP__MESSAGETYPESUPPORT_HPP_

#include "rosidl_typesupport_zenoh_cpp/message_type_support.h"
#include "TypeSupport.hpp"

namespace rmw_zenoh_cpp
{

class MessageTypeSupport : public TypeSupport
{
public:
  explicit MessageTypeSupport(const message_type_support_callbacks_t * members);
};

}  // namespace rmw_zenoh_cpp

#endif  // RMW_ZENOH_CPP__MESSAGETYPESUPPORT_HPP_
