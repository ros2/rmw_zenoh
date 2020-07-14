#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/event.h"

extern "C"
{

rmw_wait_set_t *
rmw_create_wait_set(rmw_context_t * context, size_t max_conditions)
{
  (void)context;
  (void)max_conditions;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_create_wait_set");
  return nullptr;
}

rmw_ret_t
rmw_destroy_wait_set(rmw_wait_set_t * wait_set)
{
  (void)wait_set;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_destroy_wait_set");
  return RMW_RET_OK;
}

rmw_ret_t
rmw_wait(
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  (void)subscriptions;
  (void)guard_conditions;
  (void)services;
  (void)clients;
  (void)events;
  (void)wait_set;
  (void)wait_timeout;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_wait");
  return RMW_RET_ERROR;
}

} // extern "C"
