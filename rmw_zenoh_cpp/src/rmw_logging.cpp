#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

extern "C"
{
rmw_ret_t
rmw_set_log_severity(rmw_log_severity_t severity)
{
  (void)severity;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_set_log_severity");
  return RMW_RET_ERROR;
}
}  // extern "C"
