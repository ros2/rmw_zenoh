#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

extern "C"
{

rmw_ret_t
rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator)
{
  (void)allocator;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_init_options_init");
  init_options->impl = nullptr;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  (void)src;
  (void)dst;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_init_options_copy");
  dst->impl = nullptr;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  (void)init_options;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_init_options_fini");
  return RMW_RET_OK;
}

} // extern "C"
