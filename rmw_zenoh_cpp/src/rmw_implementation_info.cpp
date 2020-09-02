#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

#include "rmw_zenoh_cpp/identifier.hpp"

extern "C"
{
const char *
rmw_get_implementation_identifier()
{
  return eclipse_zenoh_identifier;
}

const char *
rmw_get_serialization_format()
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_get_serialization_format");
  return nullptr;
}
}  // extern "C"
