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

#include "rmw/event.h"

extern "C"
{
///==============================================================================
/// Initialize a rmw subscription event
rmw_ret_t
rmw_publisher_event_init(
  rmw_event_t * rmw_event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type)
{
  (void) rmw_event;
  (void) publisher;
  (void) event_type;
  return RMW_RET_UNSUPPORTED;
}

///==============================================================================
/// Take an event from the event handle.
rmw_ret_t
rmw_subscription_event_init(
  rmw_event_t * rmw_event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type)
{
  (void) rmw_event;
  (void) subscription;
  (void) event_type;
  return RMW_RET_UNSUPPORTED;
}

///==============================================================================
rmw_ret_t
rmw_take_event(
  const rmw_event_t * event_handle,
  void * event_info,
  bool * taken)
{
  (void) event_handle;
  (void) event_info;
  (void) taken;
  return RMW_RET_UNSUPPORTED;
}


///==============================================================================
/// Finalize an rmw_event_t.
rmw_ret_t
rmw_event_fini(rmw_event_t * event)
{
  (void) event;
  return RMW_RET_UNSUPPORTED;
}

} // extern "C"
