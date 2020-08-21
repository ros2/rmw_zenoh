#ifndef RMW_ZENOH_CPP__ZENOH_SERVICE_HPP_
#define RMW_ZENOH_CPP__ZENOH_SERVICE_HPP_

#include <unordered_map>
#include <utility>
#include <string>
#include <vector>

extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

#include "rmw/rmw.h"
#include "rmw_zenoh_cpp/TypeSupport.hpp"

struct rmw_service_data_t
{
  static void zn_request_sub_callback(const zn_sample * sample);

  // Serialized ROS request messages
  static std::unordered_map<std::string, std::vector<unsigned char> > zn_request_messages_;

  const void * request_type_support_impl_;
  const void * response_type_support_impl_;
  const char * typesupport_identifier_;

  rmw_zenoh_cpp::TypeSupport * request_type_support_;
  rmw_zenoh_cpp::TypeSupport * response_type_support_;

  ZNSession * zn_session_;
  ZNQueryable * zn_queryable_;
  ZNSubscriber * zn_request_subscriber_;
  size_t zn_response_topic_id_;

  const rmw_node_t * node_;
};

#endif // RMW_ZENOH_CPP__ZENOH_SERVICE_HPP_
