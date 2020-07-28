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
// Create a node and return a handle to that node.
//
// rmw_node_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__node__t.html
//
// In the case of Zenoh, the only relevant members are name, namespace and implementation
// identifier.
//
// Most likely we will associate a subset of the context session's publishers and subscribers to
// individual nodes, even though to Zenoh it looks like the session is the one holding on to
// all of them.
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

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr
  );
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return nullptr
  );
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

  // ASSIGN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &context->options.allocator;

  // INIT NODE =================================================================
  // NOTE(CH3): Unfortunately we have to make do with these ugly copy pasted calls until
  // rcpputils::make_scope_exit() gets ported into a ROS2 release
  //
  // TODO(CH3): Once it is, replace the repeated deallocate calls with a scope exit handler
  rmw_node_t * node = static_cast<rmw_node_t *>(
    allocator->allocate(sizeof(rmw_node_t), allocator->state)
  );
  if (!node) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_node_t");
    allocator->deallocate(node, allocator->state);
    return nullptr;
  }

  // Populate common members
  node->implementation_identifier = eclipse_zenoh_identifier;

  node->name = name;  // const char * assignment
  if (!node->name) {
    RMW_SET_ERROR_MSG("failed to allocate node name");
    allocator->deallocate(node, allocator->state);
    return nullptr;
  }

  node->namespace_ = namespace_;  // const char * assignment
  if (!node->namespace_) {
    RMW_SET_ERROR_MSG("failed to allocate node namespace");
    allocator->deallocate(node, allocator->state);
    return nullptr;
  }

  node->context = context;
  if (!node->context) {
    RMW_SET_ERROR_MSG("failed to allocate node context");
    allocator->deallocate(node, allocator->state);
    return nullptr;
  }

  // POPULATE ZENOH SPECIFIC NODE MEMBERS ======================================
  node->data = static_cast<rmw_node_impl_t *>(
    allocator->allocate(sizeof(rmw_node_impl_t), allocator->state)
  );
  if (!node->data) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_node_impl_t");
    allocator->deallocate(node->data, allocator->state);
    allocator->deallocate(node, allocator->state);
    return nullptr;
  } else {
    memset(node->data, 0, sizeof(rmw_node_impl_t));
  }

  // Create graph guard condition
  rmw_node_impl_t * node_data = static_cast<rmw_node_impl_t *>(node->data);

  node_data->graph_guard_condition_ = static_cast<rmw_guard_condition_t *>(
    allocator->allocate(sizeof(rmw_guard_condition_t), allocator->state)
  );
  node_data->graph_guard_condition_ = rmw_create_guard_condition(node->context);
  if (!node_data->graph_guard_condition_) {
    rmw_destroy_guard_condition(node_data->graph_guard_condition_);

    allocator->deallocate(node_data->graph_guard_condition_, allocator->state);
    allocator->deallocate(node->data, allocator->state);
    allocator->deallocate(node, allocator->state);
    return nullptr;
  }

  // NOTE(CH3): Only for DDS
  // node_impl->domain_id_ = domain_id;

  // NOTE(CH3) TODO(CH3): No graph updates are implemented yet
  // I am not sure how this will work with Zenoh

  // TODO(CH3): For when we need to have a guard condition
  // node_impl->graph_guard_condition = rmw_create_guard_condition(context);
  // if (!node_impl->graph_guard_condition) {
  //   RMW_SET_ERROR_MSG("node implementation graph_guard_condition already set");
  //   return nullptr;
  // }

  return node;
}

/// DESTROY NODE ===============================================================
// Finalize a given node handle, reclaim the resources, and deallocate the node handle.
rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );

  // NOTE(CH3) TODO(CH3): Again, no graph updates are implemented yet
  // I am not sure how this will work with Zenoh

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  allocator->deallocate(node->data, allocator->state);
  allocator->deallocate(node, allocator->state);

  return RMW_RET_OK;
}

} // extern "C"
