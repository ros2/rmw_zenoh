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

using TypeSupport_cpp = rmw_zenoh_cpp::TypeSupport;
using MessageTypeSupport_cpp = rmw_zenoh_cpp::MessageTypeSupport;
using RequestTypeSupport_cpp = rmw_zenoh_cpp::RequestTypeSupport;
using ResponseTypeSupport_cpp = rmw_zenoh_cpp::ResponseTypeSupport;

// inline std::string
// _create_type_name(
//   const message_type_support_callbacks_t * members)
// {
//   if (!members) {
//     RMW_SET_ERROR_MSG("members handle is null");
//     return "";
//   }
//
//   std::ostringstream ss;
//   std::string message_namespace(members->message_namespace_);
//   std::string message_name(members->message_name_);
//
//   if (!message_namespace.empty()) {
//     ss << message_namespace << "::";
//   }
//   ss << "zenoh_::" << message_name << "_";
//   return ss.str();
// }

#endif  // TYPE_SUPPORT_COMMON_HPP_
