#ifndef TYPE_SUPPORT_COMMON_HPP_
#define TYPE_SUPPORT_COMMON_HPP_

#include <sstream>
#include <string>

#include "rmw/error_handling.h"

#include "rmw_zenoh_cpp/TypeSupport.hpp"
#include "rmw_zenoh_cpp/MessageTypeSupport.hpp"
#include "rmw_zenoh_cpp/ServiceTypeSupport.hpp"

#include "rmw_zenoh_cpp/identifier.hpp"

#include "rosidl_typesupport_zenoh_c/identifier.h"
#include "rosidl_typesupport_zenoh_cpp/identifier.hpp"
#include "rosidl_typesupport_zenoh_cpp/message_type_support.h"
#include "rosidl_typesupport_zenoh_cpp/service_type_support.h"

#define RMW_ZENOH_CPP_TYPESUPPORT_C rosidl_typesupport_zenoh_c__identifier
#define RMW_ZENOH_CPP_TYPESUPPORT_CPP rosidl_typesupport_zenoh_cpp::typesupport_identifier

#endif  // TYPE_SUPPORT_COMMON_HPP_
