#include "rcutils/logging_macros.h"

#include <rmw/validate_full_topic_name.h>
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/rmw.h"

#include "rmw_zenoh_cpp/rmw_context_impl.hpp"
#include "rmw_zenoh_cpp/zenoh_pubsub.hpp"
#include "rmw_zenoh_cpp/identifier.hpp"

#include "type_support_common.hpp"

extern "C"
{
#include "zenoh/zenoh-ffi.h"

/// PUBLISH ROS MESSAGE ========================================================
// Serialize and publish a ROS message using Zenoh.
rmw_ret_t
rmw_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)ros_message;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publish");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)serialized_message;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publish_serialized_message");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)ros_message;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publish_loaned_message");
  return RMW_RET_ERROR;
}

} // extern "C"
