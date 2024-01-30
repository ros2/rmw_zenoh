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

#ifndef DETAIL__EVENT_HPP_
#define DETAIL__EVENT_HPP_

#include <unordered_map>

#include "rmw/event.h"

///=============================================================================
// RMW Event types that we support in rmw_zenoh.
enum rmw_zenoh_event_type_t
{
  // sentinel value
  ZENOH_EVENT_INVALID,

  // subscription events
  ZENOH_EVENT_REQUESTED_QOS_INCOMPATIBLE,
  // RMW_EVENT_MESSAGE_LOST,
  // RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE,
  // RMW_EVENT_SUBSCRIPTION_MATCHED,

  // publisher events
  // RMW_EVENT_LIVELINESS_LOST,
  // RMW_EVENT_OFFERED_DEADLINE_MISSED,
  ZENOH_EVENT_OFFERED_QOS_INCOMPATIBLE
  // RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE,
  // RMW_EVENT_PUBLICATION_MATCHED,
};

/// Helper value to indicate the maximum index of events supported.
#define ZENOH_EVENT_ID_MAX rmw_zenoh_event_type_t::ZENOH_EVENT_OFFERED_QOS_INCOMPATIBLE

static const std::unordered_map<rmw_event_type_t, rmw_zenoh_event_type_t> event_map{
  {RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE, ZENOH_EVENT_REQUESTED_QOS_INCOMPATIBLE},
  {RMW_EVENT_OFFERED_QOS_INCOMPATIBLE, ZENOH_EVENT_OFFERED_QOS_INCOMPATIBLE},
  // TODO(clalancette): Implement remaining events
};

#endif  // DETAIL__EVENT_HPP_
