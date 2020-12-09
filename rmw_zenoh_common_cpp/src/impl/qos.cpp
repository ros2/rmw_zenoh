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


#include <rmw/types.h>


namespace rmw_zenoh_common_cpp
{
bool is_valid_qos(const rmw_qos_profile_t * qos_profile)
{
  // Depth being RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT is OK
  // Deadline being RMW_QOS_DEADLINE_DEFAULT is OK
  // Lifespan being RMW_QOS_LIFESPAN_DEFAULT is OK
  // Liveliness lease duration being RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT is OK
  return qos_profile->history != RMW_QOS_POLICY_HISTORY_UNKNOWN &&
         qos_profile->reliability != RMW_QOS_POLICY_RELIABILITY_UNKNOWN &&
         qos_profile->durability != RMW_QOS_POLICY_DURABILITY_UNKNOWN &&
         qos_profile->liveliness != RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
}
}  // namespace rmw_zenoh_common_cpp
