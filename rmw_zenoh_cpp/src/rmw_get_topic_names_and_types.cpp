#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/rmw.h"
#include "rmw/names_and_types.h"
#include "rmw/topic_endpoint_info_array.h"

extern "C"
{
rmw_ret_t
rmw_get_topic_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  (void)node;
  (void)allocator;
  (void)no_demangle;
  (void)topic_names_and_types;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_get_topic_names_and_types");
  return RMW_RET_ERROR;
}
}  // extern "C"
