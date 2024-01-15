// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "rmw/error_handling.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

#include "rmw_dds_common/qos.hpp"

extern "C"
{
rmw_ret_t
rmw_qos_profile_check_compatible(
  const rmw_qos_profile_t publisher_profile,
  const rmw_qos_profile_t subscription_profile,
  rmw_qos_compatibility_type_t * compatibility,
  char * reason,
  size_t reason_size)
{

  // In Zenoh, publishers do not have any reliability settings.
  // A publisher and subscription are only incompatible if the durability of the publisher is
  // TRANSIENT_LOCAL but that of the subscription is not. In such a scenario, a late-joining
  // subscription can fail to receive messages so we flag it accordingly.
  // However, we can reuse the qos_profile_check_compatible() method from rmw_dds_common
  // since it largely applies in rmw_zenoh.
  return rmw_dds_common::qos_profile_check_compatible(
    publisher_profile, subscription_profile, compatibility, reason, reason_size);
}
}  // extern "C"
