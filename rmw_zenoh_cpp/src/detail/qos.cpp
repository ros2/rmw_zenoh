// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "qos.hpp"

// Define defaults for various QoS settings.
#define RMW_ZENOH_DEFAULT_HISTORY RMW_QOS_POLICY_HISTORY_KEEP_LAST;
// If the depth field in the qos profile is set to 0, the RMW implementation
// has the liberty to assign a default depth. The zenoh transport protocol
// is configured with 256 channels so theoretically, this would be the maximum
// depth we can set before blocking transport. A high depth would increase the
// memory footprint of processes as more messages are stored in memory while a
// very low depth might unintentionally drop messages leading to a poor
// out-of-the-box experience for new users. For now we set the depth to 42,
// a popular "magic number". See https://en.wikipedia.org/wiki/42_(number).
#define RMW_ZENOH_DEFAULT_HISTORY_DEPTH 42;
#define RMW_ZENOH_DEFAULT_RELIABILITY RMW_QOS_POLICY_RELIABILITY_RELIABLE;
#define RMW_ZENOH_DEFAULT_DURABILITY RMW_QOS_POLICY_DURABILITY_VOLATILE;
#define RMW_ZENOH_DEFAULT_DEADLINE RMW_DURATION_INFINITE;
#define RMW_ZENOH_DEFAULT_LIFESPAN RMW_DURATION_INFINITE;
#define RMW_ZENOH_DEFAULT_LIVELINESS RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
#define RMW_ZENOH_DEFAULT_LIVELINESS_LEASE_DURATION RMW_DURATION_INFINITE;

namespace rmw_zenoh_cpp
{
///=============================================================================
QoS::QoS()
{
  default_qos_.history = RMW_ZENOH_DEFAULT_HISTORY;
  default_qos_.depth = RMW_ZENOH_DEFAULT_HISTORY_DEPTH;
  default_qos_.reliability = RMW_ZENOH_DEFAULT_RELIABILITY;
  default_qos_.durability = RMW_ZENOH_DEFAULT_DURABILITY;
  default_qos_.deadline = RMW_ZENOH_DEFAULT_DEADLINE;
  default_qos_.lifespan = RMW_ZENOH_DEFAULT_LIFESPAN;
  default_qos_.liveliness = RMW_ZENOH_DEFAULT_LIVELINESS;
  default_qos_.liveliness_lease_duration = RMW_ZENOH_DEFAULT_LIVELINESS_LEASE_DURATION;
}

///=============================================================================
QoS & QoS::get()
{
  static QoS qos;
  return qos;
}

///=============================================================================
const rmw_qos_profile_t & QoS::default_qos() const
{
  return default_qos_;
}

///=============================================================================
bool QoS::is_supported(const rmw_qos_profile_t & qos_profile)
{
  if (qos_profile.history == RMW_QOS_POLICY_HISTORY_UNKNOWN ||
    qos_profile.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN ||
    qos_profile.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN ||
    qos_profile.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN)
  {
    return false;
  }

  return true;
}

///=============================================================================
rmw_ret_t QoS::best_available_qos(
  const rmw_node_t * node,
  const char * topic_name,
  rmw_qos_profile_t * qos_profile,
  const GetEndpointInfoByTopicFunction & get_endpoint_info_for_other) const
{
  // We could rely on the GetEndpointInfoByTopicFunction callback to get
  // endpoint information of downstream consumers to match certain QoS settings.
  // Practically since Zenoh transport will succeed for any combination of
  // settings, this could only be useful to avoid creating transient_local
  // subscriptions when there are no transient_local publishers in the graph.
  // This will avoid some overhead with having QueryingSubscriber. For now,
  // we skip this optimization.
  static_cast<void>(node);
  static_cast<void>(topic_name);
  static_cast<void>(get_endpoint_info_for_other);

  if (!is_supported(*qos_profile)) {
    return RMW_RET_UNSUPPORTED;
  }

  switch (qos_profile->history) {
    case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_HISTORY_UNKNOWN:
      qos_profile->history = default_qos_.history;
    default:
      break;
  }

  if (qos_profile->depth == 0) {
    qos_profile->depth = default_qos_.depth;
  }

  switch (qos_profile->reliability) {
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_RELIABILITY_UNKNOWN:
      qos_profile->reliability = default_qos_.reliability;
    default:
      break;
  }

  switch (qos_profile->durability) {
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_DURABILITY_UNKNOWN:
      qos_profile->durability = default_qos_.durability;
    default:
      break;
  }

  if (rmw_time_equal(qos_profile->deadline, RMW_QOS_DEADLINE_DEFAULT)) {
    qos_profile->deadline = default_qos_.deadline;
  }

  if (rmw_time_equal(qos_profile->lifespan, RMW_QOS_LIFESPAN_DEFAULT)) {
    qos_profile->lifespan = default_qos_.lifespan;
  }

  if (rmw_time_equal(
      qos_profile->liveliness_lease_duration,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT))
  {
    qos_profile->liveliness_lease_duration = default_qos_.liveliness_lease_duration;
  }

  switch (qos_profile->liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT:
    case RMW_QOS_POLICY_LIVELINESS_UNKNOWN:
      qos_profile->liveliness = default_qos_.liveliness;
    default:
      break;
  }

  return RMW_RET_OK;
}
}  // namespace rmw_zenoh_cpp
