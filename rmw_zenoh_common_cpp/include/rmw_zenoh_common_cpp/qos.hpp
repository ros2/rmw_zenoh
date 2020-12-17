// Copyright 2019 Open Source Robotics Foundation, Inc.
// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef RMW_FASTRTPS_SHARED_CPP__QOS_HPP_
#define RMW_FASTRTPS_SHARED_CPP__QOS_HPP_

#include "fastrtps/attributes/PublisherAttributes.h"
#include "fastrtps/attributes/SubscriberAttributes.h"
#include "fastrtps/qos/QosPolicies.h"
#include "fastrtps/qos/ReaderQos.h"
#include "fastrtps/qos/WriterQos.h"

#include "rmw/rmw.h"

#include "rmw_fastrtps_shared_cpp/visibility_control.h"

namespace eprosima
{
namespace fastrtps
{
class SubscriberAttributes;
class PublisherAttributes;
}  // namespace fastrtps
}  // namespace eprosima

RMW_FASTRTPS_SHARED_CPP_PUBLIC
bool
is_valid_qos(const rmw_qos_profile_t & qos_policies);

RMW_FASTRTPS_SHARED_CPP_PUBLIC
bool
get_datareader_qos(
  const rmw_qos_profile_t & qos_policies,
  eprosima::fastrtps::SubscriberAttributes & sattr);

RMW_FASTRTPS_SHARED_CPP_PUBLIC
bool
get_datawriter_qos(
  const rmw_qos_profile_t & qos_policies,
  eprosima::fastrtps::PublisherAttributes & pattr);

/*
 * Converts the low-level QOS Policy; of type WriterQos or ReaderQos into rmw_qos_profile_t.
 * Since WriterQos or ReaderQos does not have information about history and depth, these values are not set
 * by this function.
 *
 * \param[in] dds_qos of type WriterQos or ReaderQos
 * \param[out] qos the equivalent of the data in WriterQos or ReaderQos in rmw_qos_profile_t
 */
template<typename DDSQoSPolicyT>
void
dds_qos_to_rmw_qos(
  const DDSQoSPolicyT & dds_qos,
  rmw_qos_profile_t * qos)
{
  switch (dds_qos.m_reliability.kind) {
    case eprosima::fastrtps::BEST_EFFORT_RELIABILITY_QOS:
      qos->reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      break;
    case eprosima::fastrtps::RELIABLE_RELIABILITY_QOS:
      qos->reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
      break;
    default:
      qos->reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
      break;
  }

  switch (dds_qos.m_durability.kind) {
    case eprosima::fastrtps::TRANSIENT_LOCAL_DURABILITY_QOS:
      qos->durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
      break;
    case eprosima::fastrtps::VOLATILE_DURABILITY_QOS:
      qos->durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
      break;
    default:
      qos->durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
      break;
  }

  qos->deadline.sec = dds_qos.m_deadline.period.seconds;
  qos->deadline.nsec = dds_qos.m_deadline.period.nanosec;

  qos->lifespan.sec = dds_qos.m_lifespan.duration.seconds;
  qos->lifespan.nsec = dds_qos.m_lifespan.duration.nanosec;

  switch (dds_qos.m_liveliness.kind) {
    case eprosima::fastrtps::AUTOMATIC_LIVELINESS_QOS:
      qos->liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
      break;
    case eprosima::fastrtps::MANUAL_BY_TOPIC_LIVELINESS_QOS:
      qos->liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
      break;
    default:
      qos->liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
      break;
  }
  qos->liveliness_lease_duration.sec = dds_qos.m_liveliness.lease_duration.seconds;
  qos->liveliness_lease_duration.nsec = dds_qos.m_liveliness.lease_duration.nanosec;
}

template<typename AttributeT>
void
dds_attributes_to_rmw_qos(
  const AttributeT & dds_qos,
  rmw_qos_profile_t * qos);

extern template RMW_FASTRTPS_SHARED_CPP_PUBLIC
void
dds_attributes_to_rmw_qos<eprosima::fastrtps::PublisherAttributes>(
  const eprosima::fastrtps::PublisherAttributes & dds_qos,
  rmw_qos_profile_t * qos);

extern template RMW_FASTRTPS_SHARED_CPP_PUBLIC
void
dds_attributes_to_rmw_qos<eprosima::fastrtps::SubscriberAttributes>(
  const eprosima::fastrtps::SubscriberAttributes & dds_qos,
  rmw_qos_profile_t * qos);

#endif  // RMW_FASTRTPS_SHARED_CPP__QOS_HPP_
