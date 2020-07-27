#ifndef RMW_ZENOH_CPP__ZENOH_PUBSUB_HPP_
#define RMW_ZENOH_CPP__ZENOH_PUBSUB_HPP_

extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

#include "rmw_zenoh_cpp/TypeSupport.hpp"

struct rmw_node_impl_t
{

};

struct rmw_publisher_data_t
{
  const void * type_support_impl_;
  const char * typesupport_identifier_;

  rmw_zenoh_cpp::TypeSupport * type_support_;

  size_t zn_topic_id_;
  ZNSession * zn_session_;
};

#endif // RMW_ZENOH_CPP__ZENOH_PUBSUB_HPP_
