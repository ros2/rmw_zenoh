#ifndef IMPL__SERVICE_IMPL_HPP_
#define IMPL__SERVICE_IMPL_HPP_

#include <unordered_map>
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

struct rmw_service_data_t
{
  /// STATIC MEMBERS ===========================================================
  static void zn_request_sub_callback(const zn_sample * sample);

  // Counter to give service servers unique IDs
  static std::atomic<size_t> service_id_counter;

  // Map of Zenoh topic key expression to service data struct instances
  static std::unordered_map<std::string, std::vector<rmw_service_data_t *>>
    zn_topic_to_service_data;

  /// INSTANCE MEMBERS =========================================================
  // Type support
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

  // Instanced request message queue
  std::deque<std::shared_ptr<std::vector<unsigned char>>> zn_request_message_queue_;
  std::mutex request_queue_mutex_;

  size_t service_id_;
  size_t queue_depth_;
};

#endif  // IMPL__SERVICE_IMPL_HPP_
