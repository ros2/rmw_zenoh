#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/event.h"

extern "C"
{

rmw_ret_t
rmw_take_event(const rmw_event_t * event_handle, void * event_info, bool * taken)
{
  (void)event_handle;
  (void)event_info;
  (void)taken;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take_event");
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_event_init(
  rmw_event_t * event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type)
{
  (void)event;
  (void)publisher;
  (void)event_type;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publisher_event_init");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_subscription_event_init(
  rmw_event_t * event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type)
{
  (void)event;
  (void)subscription;
  (void)event_type;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_subscription_event_init");
  return RMW_RET_ERROR;
}

} // extern "C"
