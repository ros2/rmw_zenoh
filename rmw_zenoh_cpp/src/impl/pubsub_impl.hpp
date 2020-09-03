#ifndef IMPL__PUBSUB_IMPL_HPP_
#define IMPL__PUBSUB_IMPL_HPP_

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

  const rmw_node_t * node_;
};

// Functionally a struct. But with a method for handling incoming Zenoh messages
struct rmw_subscription_data_t
{
  /// STATIC MEMBERS ===============================================================================
  static void zn_sub_callback(const zn_sample * sample);

  // Counter to give subscriptions unique IDs
  static std::atomic<size_t> subscription_id_counter;

  // Map of Zenoh topic key expression to subscription data struct instances
  static std::unordered_map<std::string, std::vector<rmw_subscription_data_t *> >
    zn_topic_to_sub_data;

  /// INSTANCE MEMBERS =============================================================================
  const void * type_support_impl_;
  const char * typesupport_identifier_;

  rmw_zenoh_cpp::TypeSupport * type_support_;
  const rmw_node_t * node_;

  ZNSession * zn_session_;
  ZNSubscriber * zn_subscriber_;

  // Instanced message queue
  std::deque<std::shared_ptr<std::vector<unsigned char> > > zn_message_queue_;
  std::mutex message_queue_mutex_;

  size_t subscription_id_;
  size_t queue_depth_;
};

#endif  // IMPL__PUBSUB_IMPL_HPP_
