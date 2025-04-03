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

#ifndef DETAIL__QOS_HPP_
#define DETAIL__QOS_HPP_

#include <rcutils/logging.h>
#include <rmw/topic_endpoint_info_array.h>
#include <rmw/types.h>

#include <functional>

namespace rmw_zenoh_cpp
{
//==============================================================================
/// Signature matching rmw_get_publishers_info_by_topic and rmw_get_subscriptions_info_by_topic
using GetEndpointInfoByTopicFunction = std::function<rmw_ret_t(
      const rmw_node_t *,
      rcutils_allocator_t *,
      const char *,
      bool,
      rmw_topic_endpoint_info_array_t *)>;

//==============================================================================
class QoS
{
public:
  static QoS & get();

  const rmw_qos_profile_t & default_qos() const;

  /// Returns true if the qos profile is supported by rmw_zenoh.
  static bool is_supported(const rmw_qos_profile_t & qos_profile);

  rmw_ret_t best_available_qos(
    const rmw_node_t * node,
    const char * topic_name,
    rmw_qos_profile_t * qos_profile,
    const GetEndpointInfoByTopicFunction & get_endpoint_info_for_other) const;

private:
  // Private constructor which initializes the default qos.
  QoS();
  rmw_qos_profile_t default_qos_;
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__QOS_HPP_
