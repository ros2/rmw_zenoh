#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/rmw.h"

extern "C"
{

rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_publisher_allocation_t * allocation)
{
  (void)type_support;
  (void)message_bounds;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_init_publisher_allocation");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_fini_publisher_allocation(rmw_publisher_allocation_t * allocation)
{
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_fini_publisher_allocation");
  return RMW_RET_ERROR;
}

rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options)
{
  (void)node;
  (void)type_support;
  (void)topic_name;
  (void)qos_profile;
  (void)publisher_options;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_create_publisher");
  return nullptr;
}

rmw_ret_t
rmw_destroy_publisher(rmw_node_t * node, rmw_publisher_t * publisher)
{
  (void)node;
  (void)publisher;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_destroy_publisher");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message)
{
  (void)publisher;
  (void)type_support;
  (void)ros_message;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_borrow_loaned_message");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_return_loaned_message_from_publisher(const rmw_publisher_t * publisher, void * loaned_message)
{
  (void)publisher;
  (void)loaned_message;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_return_loaned_message_from_publisher");
  return RMW_RET_ERROR;
}

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

rmw_ret_t
rmw_publisher_count_matched_subscriptions(const rmw_publisher_t * publisher, size_t * count)
{
  (void)publisher;
  (void)count;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publisher_count_matched_subscriptions");
  return RMW_RET_ERROR;
}

// STUB
rmw_ret_t
rmw_publisher_get_actual_qos(const rmw_publisher_t * publisher, rmw_qos_profile_t * qos_profile)
{
  (void)publisher;
  (void)qos_profile;
  // RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publisher_get_actual_qos");
  // return RMW_RET_ERROR;
  return RMW_RET_OK;
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
rmw_publisher_assert_liveliness(const rmw_publisher_t * publisher)
{
  (void)publisher;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publisher_assert_liveliness");
  return RMW_RET_ERROR;
}

} // extern "C"
