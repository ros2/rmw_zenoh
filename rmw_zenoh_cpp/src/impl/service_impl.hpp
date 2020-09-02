#ifndef IMPL__SERVICE_IMPL_HPP_
#define IMPL__SERVICE_IMPL_HPP_

#include <unordered_map>
#include <utility>
#include <string>
#include <vector>

#include "rmw/rmw.h"
#include "rmw_zenoh_cpp/TypeSupport.hpp"

extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

struct rmw_service_data_t
{
  /// STATIC MEMBERS ===========================================================
  static void zn_request_sub_callback(const zn_sample * sample);

  // Serialized ROS request messages
  static std::unordered_map<std::string, std::vector<unsigned char> > zn_request_messages_;

  /// TYPE SUPPORT =============================================================
  const void * request_type_support_impl_;
  const void * response_type_support_impl_;
  const char * typesupport_identifier_;

  rmw_zenoh_cpp::TypeSupport * request_type_support_;
  rmw_zenoh_cpp::TypeSupport * response_type_support_;

  /// ZENOH ====================================================================
  ZNSession * zn_session_;
  ZNQueryable * zn_queryable_;

  // Request Sub
  const char * zn_request_topic_key_;
  ZNSubscriber * zn_request_subscriber_;

  // Response Pub
  const char * zn_response_topic_key_;
  size_t zn_response_topic_id_;

  /// ROS ======================================================================
  const rmw_node_t * node_;
};

#endif  // IMPL__SERVICE_IMPL_HPP_
