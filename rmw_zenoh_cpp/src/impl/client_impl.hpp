#ifndef IMPL__CLIENT_IMPL_HPP_
#define IMPL__CLIENT_IMPL_HPP_

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <string>
#include <vector>
#include <memory>
#include <deque>
#include <mutex>
#include <atomic>

#include "rmw/rmw.h"
#include "rmw_zenoh_cpp/TypeSupport.hpp"

extern "C"
{
  #include "zenoh/zenoh-ffi.h"
}

struct rmw_client_data_t
{
  /// STATIC MEMBERS ===========================================================
  static void zn_response_sub_callback(const zn_sample * sample);
  static void zn_service_availability_query_callback(
    const zn_source_info * info, const zn_sample * sample
  );

  // Sequence id
  static std::atomic<std::int64_t> sequence_id_counter;

  // Serialized ROS response messages
  static std::unordered_map<std::string, std::vector<unsigned char>> zn_response_messages_;

  // Availability query Zenoh responses
  static std::unordered_set<std::string> zn_availability_query_responses_;

  /// TYPE SUPPORT =============================================================
  const void * request_type_support_impl_;
  const void * response_type_support_impl_;
  const char * typesupport_identifier_;

  rmw_zenoh_cpp::TypeSupport * request_type_support_;
  rmw_zenoh_cpp::TypeSupport * response_type_support_;

  /// ZENOH ====================================================================
  ZNSession * zn_session_;

  // Response Sub
  const char * zn_response_topic_key_;
  ZNSubscriber * zn_response_subscriber_;

  // Request Pub
  const char * zn_request_topic_key_;
  size_t zn_request_topic_id_;

  /// ROS ======================================================================
  const rmw_node_t * node_;
};

#endif  // IMPL__CLIENT_IMPL_HPP_
