// Doc: http://docs.ros2.org/latest/api/rmw/rmw_8h.html

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/validate_node_name.h"
#include "rmw/validate_namespace.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"

#include "rmw_zenoh_cpp/identifier.hpp"
#include "rmw_zenoh_cpp/rmw_init_options_impl.hpp"
#include "rmw_zenoh_cpp/rmw_context_impl.hpp"
#include "rmw_zenoh_cpp/rmw_node_impl.hpp"

extern "C"
{

/// CREATE NODE ================================================================
// rmw_node_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__node__t.html
rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_,
  size_t domain_id,
  bool localhost_only)
{
  (void)domain_id;
  (void)localhost_only;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_create_node");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return nullptr);
  if (context->impl->is_shutdown) {
    RCUTILS_SET_ERROR_MSG("context has been shutdown");
    return nullptr;
  }

  // Validate node name
  int validation_result = RMW_NODE_NAME_VALID;
  rmw_ret_t ret = rmw_validate_node_name(name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  if (RMW_NODE_NAME_VALID != validation_result) {
    const char * reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node name: %s", reason);
    return nullptr;
  }

  // Validate namespace
  validation_result = RMW_NAMESPACE_VALID;
  ret = rmw_validate_namespace(namespace_, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  if (RMW_NAMESPACE_VALID != validation_result) {
    const char * reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node namespace: %s", reason);
    return nullptr;
  }

  // CLEANUP DEFINITIONS =======================================================
  // Store a pointer to the node with an exit handler that destroys the node if
  // any initialization steps fail
  rmw_node_t * node_handle = nullptr;
  std::unique_ptr<rmw_node_t, void (*)(rmw_node_t *)> clean_when_fail(
    node_handle,
    [](rmw_node_t * node_handle)
    {
      rmw_destroy_node(node_handle);
    });

  // INIT NODE =================================================================
  node_handle = rmw_node_allocate();
  if (!node_handle) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_node_t");
    return nullptr;
  }

  // Populate common members
  node_handle->implementation_identifier = eclipse_zenoh_identifier;

  node_handle->name = rcutils_strdup(name, context->options.allocator);
  if (!node_handle->name) {
    RMW_SET_ERROR_MSG("failed to allocate node name");
    return nullptr;
  }

  node_handle->namespace_ = rcutils_strdup(namespace_, context->options.allocator);
  if (!node_handle->namespace_) {
    RMW_SET_ERROR_MSG("failed to allocate node namespace");
    return nullptr;
  }

  // POPULATE ZENOH SPECIFIC NODE MEMBERS ======================================
  std::unique_ptr<rmw_node_impl_t> node_impl(new (std::nothrow) rmw_node_impl_t());
  if (!node_impl) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_node_impl_t");
    return nullptr;
  }

  // NOTE(CH3): Only for DDS
  // node_impl->domain_id_ = domain_id;

  // TODO(CH3): For when we need to have a guard condition
  // node_impl->graph_guard_condition = rmw_create_guard_condition(context);
  // if (!node_impl->graph_guard_condition) {
  //   RMW_SET_ERROR_MSG("node implementation graph_guard_condition already set");
  //   return nullptr;
  // }

  node_handle->data = node_impl.release();
  clean_when_fail.release();
  return node_handle;
}

/// DESTROY NODE ===============================================================
rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  (void)node;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_destroy_node");
  return RMW_RET_ERROR;
}

} // extern "C"
