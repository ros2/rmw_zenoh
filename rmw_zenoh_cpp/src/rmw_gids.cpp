#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/names_and_types.h"
#include "rmw/topic_endpoint_info_array.h"

extern "C"
{

rmw_ret_t
rmw_get_gid_for_publisher(const rmw_publisher_t * publisher, rmw_gid_t * gid)
{
  (void)publisher;
  (void)gid;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_get_gid_for_publisher");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_compare_gids_equal(const rmw_gid_t * gid1, const rmw_gid_t * gid2, bool * result)
{
  (void)gid1;
  (void)gid2;
  (void)result;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_compare_gids_equal");
  return RMW_RET_ERROR;
}

} // extern "C"
