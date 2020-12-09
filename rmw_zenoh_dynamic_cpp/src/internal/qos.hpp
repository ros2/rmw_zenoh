// Copyright 2020 Continental AG
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

#pragma once

#include <limits>
#include <stdexcept>

#include <ecal/ecal.h>

#include <rmw/types.h>

namespace eCAL
{
  namespace rmw
  {

    static const std::string pub_name_prefix{ "rt" };
    static const std::string service_name_prefix{ "rs" };
    static const std::string parameter_name_prefix{ "rp" };
    static const std::string action_name_prefix{ "ra" };
    static const std::string private_symbol_prefix{ "_" };
    static const std::string node_query_service_prefix{ service_name_prefix + "/" + private_symbol_prefix + "node" };

    inline std::string DemangleTopicName(const std::string& topic_name)
    {
      if (topic_name.substr(0, 3) == pub_name_prefix + "/")
        return topic_name.substr(3);
      return topic_name;
    }

    inline std::string DemangleServiceName(const std::string& service_name)
    {
      if (service_name.substr(0, 3) == service_name_prefix + "/")
        return service_name.substr(3);
      return service_name;
    }

    inline std::string BuildQueryServiceName(const std::string& namespace_, const std::string& name)
    {
      return node_query_service_prefix + namespace_ + name;
    }

    inline int ToECalDepth(size_t depth)
    {
      int max_int{ std::numeric_limits<int>::max() };
      if (depth > static_cast<size_t>(max_int))
        throw std::logic_error{ "Maximum history depth value is: " + std::to_string(max_int) };

      return static_cast<int>(depth);
    }

    inline bool IsPolicySpecified(rmw_qos_history_policy_t history_policy)
    {
      return history_policy != RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT && history_policy != RMW_QOS_POLICY_HISTORY_UNKNOWN;
    }

    inline bool IsPolicySpecified(rmw_qos_reliability_policy_t history_policy)
    {
      return history_policy != RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT && history_policy != RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    }

    inline eCAL::QOS::eQOSPolicy_HistoryKind ToECalPolicy(rmw_qos_history_policy_t history_policy)
    {
      switch (history_policy)
      {
      case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
        return eCAL::QOS::eQOSPolicy_HistoryKind::keep_last_history_qos;
      case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
        return eCAL::QOS::eQOSPolicy_HistoryKind::keep_all_history_qos;
      default:
        throw std::invalid_argument{ "Invalid history policy value." };
      }
    }

    inline eCAL::QOS::eQOSPolicy_Reliability ToECalPolicy(rmw_qos_reliability_policy_t reliability_policy)
    {
      switch (reliability_policy)
      {
      case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
        return eCAL::QOS::eQOSPolicy_Reliability::best_effort_reliability_qos;
      case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
        return eCAL::QOS::eQOSPolicy_Reliability::reliable_reliability_qos;
      default:
        throw std::invalid_argument{ "Invalid reliability policy value." };
      }
    }

    inline rmw_qos_reliability_policy_t ToRosPolicy(eCAL::QOS::eQOSPolicy_Reliability reliability_policy)
    {
      switch (reliability_policy)
      {
      case eCAL::QOS::eQOSPolicy_Reliability::best_effort_reliability_qos:
        return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      case eCAL::QOS::eQOSPolicy_Reliability::reliable_reliability_qos:
        return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
      default:
        throw std::invalid_argument{ "Invalid reliability policy value." };
      }
    }

    inline rmw_qos_history_policy_t ToRosPolicy(eCAL::QOS::eQOSPolicy_HistoryKind history_policy)
    {
      switch (history_policy)
      {
      case eCAL::QOS::eQOSPolicy_HistoryKind::keep_last_history_qos:
        return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
      case eCAL::QOS::eQOSPolicy_HistoryKind::keep_all_history_qos:
        return RMW_QOS_POLICY_HISTORY_KEEP_ALL;
      default:
        throw std::invalid_argument{ "Invalid history policy value." };
      }
    }

    struct PublisherQOS
    {
      rmw_qos_profile_t rmw_qos;
      eCAL::QOS::SWriterQOS ecal_qos;
      std::string topic_name_prefix;
    };

    inline PublisherQOS CreatePublisherQOS(const rmw_qos_profile_t* rmw_qos)
    {
      PublisherQOS qos;
      if (IsPolicySpecified(rmw_qos->history))
      {
        qos.ecal_qos.history_kind = ToECalPolicy(rmw_qos->history);
      }
      if (IsPolicySpecified(rmw_qos->reliability))
      {
        qos.ecal_qos.reliability = ToECalPolicy(rmw_qos->reliability);
      }
      qos.ecal_qos.history_kind_depth = ToECalDepth(rmw_qos->depth);

      qos.rmw_qos.avoid_ros_namespace_conventions = rmw_qos->avoid_ros_namespace_conventions;
      qos.rmw_qos.depth = rmw_qos->depth;
      qos.rmw_qos.history = ToRosPolicy(qos.ecal_qos.history_kind);
      qos.rmw_qos.reliability = ToRosPolicy(qos.ecal_qos.reliability);
      qos.rmw_qos.deadline = { 0, 0 };
      qos.rmw_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
      qos.rmw_qos.lifespan = { 0, 0 };
      qos.rmw_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
      qos.rmw_qos.liveliness_lease_duration = { 0, 0 };

      if (!rmw_qos->avoid_ros_namespace_conventions)
      {
        qos.topic_name_prefix = pub_name_prefix;
      }

      return qos;
    }

    struct SubscriberQOS
    {
      rmw_qos_profile_t rmw_qos;
      eCAL::QOS::SReaderQOS ecal_qos;
      std::string topic_name_prefix;
    };

    inline SubscriberQOS CreateSubscriberQOS(const rmw_qos_profile_t* rmw_qos)
    {
      SubscriberQOS qos;
      if (IsPolicySpecified(rmw_qos->history))
      {
        qos.ecal_qos.history_kind = ToECalPolicy(rmw_qos->history);
      }
      if (IsPolicySpecified(rmw_qos->reliability))
      {
        qos.ecal_qos.reliability = ToECalPolicy(rmw_qos->reliability);
      }
      qos.ecal_qos.history_kind_depth = ToECalDepth(rmw_qos->depth);

      qos.rmw_qos.avoid_ros_namespace_conventions = rmw_qos->avoid_ros_namespace_conventions;
      qos.rmw_qos.depth = rmw_qos->depth;
      qos.rmw_qos.history = ToRosPolicy(qos.ecal_qos.history_kind);
      qos.rmw_qos.reliability = ToRosPolicy(qos.ecal_qos.reliability);
      qos.rmw_qos.deadline = { 0, 0 };
      qos.rmw_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
      qos.rmw_qos.lifespan = { 0, 0 };
      qos.rmw_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
      qos.rmw_qos.liveliness_lease_duration = { 0, 0 };

      if (!rmw_qos->avoid_ros_namespace_conventions)
      {
        qos.topic_name_prefix = pub_name_prefix;
      }

      return qos;
    }

    struct ServiceQOS
    {
      std::string service_name_prefix;
    };

    inline ServiceQOS CreateServiceQOS(const rmw_qos_profile_t* rmw_qos)
    {
      ServiceQOS qos;
      if (!rmw_qos->avoid_ros_namespace_conventions)
      {
        qos.service_name_prefix = service_name_prefix;
      }

      return qos;
    }

    struct ClientQOS
    {
      std::string service_name_prefix;
    };

    inline ClientQOS CreateClientQOS(const rmw_qos_profile_t* rmw_qos)
    {
      ClientQOS qos;
      if (!rmw_qos->avoid_ros_namespace_conventions)
      {
        qos.service_name_prefix = service_name_prefix;
      }

      return qos;
    }

  } // namespace rmw
} // namespace eCAL
