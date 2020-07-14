#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

extern "C"
{

rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_,
  size_t domain_id,
  bool localhost_only)
{
  (void)context;
  (void)name;
  (void)namespace_;
  (void)domain_id;
  (void)localhost_only;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_create_node");
  return nullptr;
}

rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  (void)node;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_destroy_node");
  return RMW_RET_ERROR;
}

} // extern "C"
