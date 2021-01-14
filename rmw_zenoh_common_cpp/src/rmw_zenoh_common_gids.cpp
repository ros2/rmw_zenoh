// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rcutils/logging_macros.h"

#include "rmw/topic_endpoint_info_array.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/names_and_types.h"
#include "rmw/rmw.h"

#include "impl/pubsub_impl.hpp"

#include "rmw_zenoh_common_cpp/rmw_zenoh_common.h"

/// GET PUBLISHER GID ==========================================================
// rmw_gid_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__gid__t.html
rmw_ret_t
rmw_zenoh_common_get_gid_for_publisher(
  const rmw_publisher_t * publisher,
  rmw_gid_t * gid,
  const char * const eclipse_zenoh_identifier)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);

  // Copy size_t to gid member
  memset(gid->data, 0, sizeof(gid->data));
  memcpy(
    gid->data,
    &(static_cast<rmw_publisher_data_t *>(publisher->data)->zn_topic_id_),
    sizeof(static_cast<rmw_publisher_data_t *>(publisher->data)->zn_topic_id_));

  return RMW_RET_OK;
}

rmw_ret_t
rmw_compare_gids_equal(const rmw_gid_t * gid1, const rmw_gid_t * gid2, bool * result)
{
  (void)gid1;
  (void)gid2;
  (void)result;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_common_cpp", "rmw_compare_gids_equal");
  return RMW_RET_UNSUPPORTED;
}
