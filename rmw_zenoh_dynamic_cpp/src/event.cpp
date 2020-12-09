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

#include <rmw/event.h>
#include <rmw/rmw.h>

#include "internal/publisher.hpp"
#include "internal/subscriber.hpp"
#include "internal/common.hpp"

rmw_event_t rmw_get_zero_initialized_event(void)
{
  return {};
}

rmw_ret_t rmw_publisher_event_init(rmw_event_t* rmw_event,
                                   const rmw_publisher_t* publisher,
                                   rmw_event_type_t event_type)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(publisher);

  if (event_type != rmw_event_type_t::RMW_EVENT_OFFERED_DEADLINE_MISSED)
  {
    RMW_SET_ERROR_MSG("Unsupported publisher event type.");
    return RMW_RET_UNSUPPORTED;
  }

  rmw_event->event_type = event_type;
  rmw_event->implementation_identifier = rmw_get_implementation_identifier();
  rmw_event->data = &eCAL::rmw::GetImplementation(publisher)->GetDataDroppedEventListener();

  return RMW_RET_OK;
}

rmw_ret_t rmw_subscription_event_init(rmw_event_t* rmw_event,
                                      const rmw_subscription_t* subscription,
                                      rmw_event_type_t event_type)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(subscription);

  if (event_type != rmw_event_type_t::RMW_EVENT_REQUESTED_DEADLINE_MISSED)
  {
    RMW_SET_ERROR_MSG("Unsupported publisher event type.");
    return RMW_RET_UNSUPPORTED;
  }

  rmw_event->event_type = event_type;
  rmw_event->implementation_identifier = rmw_get_implementation_identifier();
  rmw_event->data = &eCAL::rmw::GetImplementation(subscription)->GetDataDroppedEventListener();

  return RMW_RET_OK;
}

rmw_ret_t rmw_take_event(const rmw_event_t* event_handle,
                         void* /* event_info */,
                         bool* taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(event_handle, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(event_handle);

  auto ecal_event = eCAL::rmw::GetImplementation(event_handle);
  *taken = ecal_event->TakeTriggered();

  return RMW_RET_OK;
}

rmw_ret_t rmw_event_fini(rmw_event_t* event)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(event, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(event);

  event->event_type = rmw_event_type_t::RMW_EVENT_INVALID;
  event->implementation_identifier = nullptr;
  delete eCAL::rmw::GetImplementation(event);

  return RMW_RET_OK;
}
