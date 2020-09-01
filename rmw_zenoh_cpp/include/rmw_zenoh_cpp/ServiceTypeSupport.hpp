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
