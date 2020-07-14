#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

extern "C"
{

rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  (void)options;
  (void)context;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_init");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  (void)context;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_shutdown");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  (void)context;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_context_fini");
  return RMW_RET_ERROR;
}

} // extern "C"
