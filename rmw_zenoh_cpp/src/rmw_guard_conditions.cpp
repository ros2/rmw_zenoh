#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/event.h"

extern "C"
{

rmw_guard_condition_t *
rmw_create_guard_condition(rmw_context_t * context)
{
  (void)context;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_create_guard_condition");
  return nullptr;
}

rmw_ret_t
rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition_handle)
{
  (void)guard_condition_handle;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_destroy_guard_condition");
  return RMW_RET_OK;
}

rmw_ret_t
rmw_trigger_guard_condition(const rmw_guard_condition_t * guard_condition_handle)
{
  (void)guard_condition_handle;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_trigger_guard_condition");
  return RMW_RET_OK;
}

const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(const rmw_node_t * node)
{
  (void)node;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_node_get_graph_guard_condition");
  return nullptr;
}

} // extern "C"
