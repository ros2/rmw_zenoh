// Docs: http://docs.ros2.org/latest/api/rmw/event_8h.html

#include "rcutils/logging_macros.h"

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/event.h"

#include "rmw_zenoh_cpp/identifier.hpp"

extern "C"
{
// STUB
rmw_ret_t
rmw_take_event(const rmw_event_t * event_handle, void * event_info, bool * taken)
{
  (void)event_handle;
  (void)event_info;

  // RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take_event (STUB)");
  *taken = true;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_event_init(
  rmw_event_t * event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type)
{
  // RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publisher_event_init (STUB)");

  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(event, RMW_RET_INVALID_ARGUMENT);

  if (event_type == RMW_EVENT_INVALID)
    return RMW_RET_INVALID_ARGUMENT;

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(publisher,
                                   publisher->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // TODO(CH3) NOTE(CH3): Check if event type is supported
  // Most likely no. It seems to be a DDS QoS specific thing
  // if (!internal::is_event_supported(event_type)) {
  //   RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("provided event_type is not supported by %s", identifier);
  //   return RMW_RET_UNSUPPORTED;
  // }

  event->implementation_identifier = publisher->implementation_identifier;
  event->data = publisher->data;
  event->event_type = event_type;

  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_event_init(
  rmw_event_t * event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type)
{
  // RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_subscription_event_init");

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(event, RMW_RET_INVALID_ARGUMENT);

  if (event_type == RMW_EVENT_INVALID)
    return RMW_RET_INVALID_ARGUMENT;

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );

  // TODO(CH3) NOTE(CH3): Check if event type is supported
  // Most likely no. It seems to be a DDS QoS specific thing
  // if (!internal::is_event_supported(event_type)) {
  //   RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("provided event_type is not supported by %s", identifier);
  //   return RMW_RET_UNSUPPORTED;
  // }

  event->implementation_identifier = subscription->implementation_identifier;
  event->data = subscription->data;
  event->event_type = event_type;

  return RMW_RET_OK;
}

} // extern "C"
