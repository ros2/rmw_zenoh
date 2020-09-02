#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/rmw.h"
#include "rmw/names_and_types.h"
#include "rmw/topic_endpoint_info_array.h"

extern "C"
{
rmw_ret_t
rmw_get_publishers_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * publishers_info)
{
  (void)node;
  (void)allocator;
  (void)topic_name;
  (void)no_mangle;
  (void)publishers_info;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_get_publishers_info_by_topic");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_get_subscriptions_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * subscribers_info)
{
  (void)node;
  (void)allocator;
  (void)topic_name;
  (void)no_mangle;
  (void)subscribers_info;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_get_subscriptions_info_by_topic");
  return RMW_RET_ERROR;
}
}  // extern "C"
