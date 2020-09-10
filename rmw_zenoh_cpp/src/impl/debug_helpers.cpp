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

#include "debug_helpers.hpp"
#include "rcutils/logging_macros.h"

void rmw_zenoh_cpp::log_debug_qos_profile(const rmw_qos_profile_t * qos_profile)
{
  const char * reliability_string = NULL;
  switch (qos_profile->reliability) {
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
      reliability_string = "system default";
      break;
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      reliability_string = "reliable";
      break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      reliability_string = "best effort";
      break;
    case RMW_QOS_POLICY_RELIABILITY_UNKNOWN:
      reliability_string = "unknown";
      break;
    default:
      reliability_string = "invalid";
      break;
  }

  const char * history_string = NULL;
  switch (qos_profile->history) {
    case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
      history_string = "system default";
      break;
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      history_string = "keep last";
      break;
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      history_string = "keep all";
      break;
    case RMW_QOS_POLICY_HISTORY_UNKNOWN:
      history_string = "unknown";
      break;
    default:
      history_string = "invalid";
      break;
  }
  const char * durability_string = NULL;
  switch (qos_profile->durability) {
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
      durability_string = "system default";
      break;
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      durability_string = "transient local";
      break;
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      durability_string = "volatile";
      break;
    case RMW_QOS_POLICY_DURABILITY_UNKNOWN:
      durability_string = "unknown";
      break;
    default:
      durability_string = "invalid";
      break;
  }

  const char * liveliness_string = NULL;
  switch (qos_profile->liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT:
      liveliness_string = "system default";
      break;
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
      liveliness_string = "automatic";
      break;

    // we want to print the debug message, but not trigger the compiler warning
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC+1:
      liveliness_string = "manual by node (deprecated)";
      break;
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
      liveliness_string = "manual by topic";
      break;
    case RMW_QOS_POLICY_LIVELINESS_UNKNOWN:
      liveliness_string = "unknown";
      break;
    default:
      liveliness_string = "invalid";
      break;
  }

  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "    reliability: %s", reliability_string);
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "    history: %s", history_string);
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "    depth: %zu", qos_profile->depth);
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "    durability: %s", durability_string);
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "    deadline: (%zu, %zu)",
    qos_profile->deadline.sec,
    qos_profile->deadline.nsec);
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "    lifespan: (%zu, %zu)",
    qos_profile->lifespan.sec,
    qos_profile->lifespan.nsec);
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "    liveliness: %s", liveliness_string);
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "    liveliness lease duration: (%zu, %zu)",
    qos_profile->liveliness_lease_duration.sec,
    qos_profile->liveliness_lease_duration.nsec);
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "    avoid ros namespace conventions: %s",
    qos_profile->avoid_ros_namespace_conventions ? "true" : "false");

}
