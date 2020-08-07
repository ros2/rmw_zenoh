#include "rcutils/logging_macros.h"

#include "rmw/topic_endpoint_info_array.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/names_and_types.h"
#include "rmw/rmw.h"

#include "rmw_zenoh_cpp/zenoh_pubsub.hpp"
#include "rmw_zenoh_cpp/identifier.hpp"

#include "rmw_zenoh_cpp/identifier.hpp"

#include "pubsub_impl.hpp"

extern "C"
{
/// GET PUBLISHER GID ==========================================================
// rmw_gid_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__gid__t.html
rmw_ret_t
rmw_get_gid_for_publisher(const rmw_publisher_t * publisher, rmw_gid_t * gid)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);

  // Copy size_t to gid member
  memset(gid->data, 0, sizeof(gid->data));
  memcpy(
    gid->data,
    &static_cast<rmw_publisher_data_t *>(publisher->data)->zn_topic_id_,
    sizeof(static_cast<rmw_publisher_data_t *>(publisher->data)->zn_topic_id_)
  );

  return RMW_RET_OK;
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
