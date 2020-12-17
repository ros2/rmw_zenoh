// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Doc: http://docs.ros2.org/latest/api/rmw/init_8h.html

#include <cstring>

#include <memory>

#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/init.h"

#include "rcutils/logging_macros.h"

#include "rmw_zenoh_common_cpp/rmw_context_impl.hpp"
#include "rmw_zenoh_common_cpp/rmw_init_options_impl.hpp"

#include "rmw_zenoh_common_cpp/identifier.hpp"

#include "rmw_zenoh_common_cpp/rmw_zenoh_common.h"
#include "rmw_zenoh_common_cpp/zenoh-net-interface.h"

zn_properties_t * configure_connection_mode(rmw_context_t * context)
{
  if (strcmp(context->options.impl->mode, "CLIENT") == 0) {
    return zn_config_client(context->options.impl->session_locator);
  } else {
    RMW_SET_ERROR_MSG("zenoh-pico can only work in client mode");
    return NULL;
  }
}

void configure_session(zn_session_t * session)
{
  // Start the read session session lease loops
  znp_start_read_task(session);
  znp_start_lease_task(session);
}

const char *
rmw_get_implementation_identifier()
{
  return eclipse_zenoh_identifier;
}

const char *
rmw_get_serialization_format()
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "rmw_get_serialization_format");
  return nullptr;
}

/// INIT CONTEXT ===============================================================
// Initialize the middleware with the given options, and yielding an context.
//
// rmw_context_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__context__t.html
//
// Starts a new Zenoh session and configures it according to the init options
// with the following environment variables:
//  - RMW_ZENOH_SESSION_LOCATOR: Session TCP locator to use
//  - RMW_ZENOH_MODE: Lets you set the session to be in CLIENT, ROUTER, or PEER mode
//                    (defaults to PEER)
rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_init");

  // CLEANUP DEFINITIONS =======================================================
  // Store a pointer to the context with a custom deleter that zero inits the
  // context if any initialization steps fail
  std::unique_ptr<rmw_context_t, void (*)(rmw_context_t *)> clean_when_fail(
    context,
    [](rmw_context_t * context) {*context = rmw_get_zero_initialized_context();});

  rmw_ret_t ret = rmw_zenoh_common_init_pre(options, context, eclipse_zenoh_identifier);
  if (ret == RMW_RET_OK) {
    // Create implementation specific context
    rcutils_allocator_t * allocator = &context->options.allocator;

    rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(
      allocator->allocate(sizeof(rmw_context_impl_t), allocator->state));

    if (!context_impl) {
      RMW_SET_ERROR_MSG("failed to allocate context impl");
      return RMW_RET_BAD_ALLOC;
    }

    // Open configured Zenoh session, then assign it to the context
    zn_properties_t * config = configure_connection_mode(context);
    if (nullptr == config) {
      allocator->deallocate(context_impl, allocator->state);
      return RMW_RET_ERROR;
    }

    zn_session_t * session = zn_open(config);

    if (session == nullptr) {
      RMW_SET_ERROR_MSG("failed to create Zenoh session when starting context");
      allocator->deallocate(context_impl, allocator->state);
      return RMW_RET_ERROR;
    } else {
      context_impl->session = session;
      context_impl->is_shutdown = false;
    }

    // CLEANUP IF PASSED =========================================================
    context->impl = context_impl;
    clean_when_fail.release();

    configure_session(context_impl->session);
  }
  return ret;
}

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
  return rmw_zenoh_common_create_node(
    context,
    name,
    namespace_,
    domain_id,
    localhost_only,
    eclipse_zenoh_identifier);
}

/// SHUTDOWN CONTEXT ===========================================================
// Shutdown the middleware for a given context.
//
// In this case, closes the associated Zenoh session.
rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  return rmw_zenoh_common_shutdown(context, eclipse_zenoh_identifier);
}

/// INIT OPTIONS ===============================================================
// Initialize given init_options with the default values
// and implementation specific values.
//
// rmw_init_options_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__init__options__t.html
//
// Note: You should call rmw_get_zero_initialized_init_options()
// to get a zero initialized rmw_init_options_t struct first
rmw_ret_t
rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator)
{
  return rmw_zenoh_common_init_options_init(init_options, allocator, eclipse_zenoh_identifier);
}

/// FINALIZE CONTEXT ===========================================================
// Finalize a context. (Cleanup and deallocation.)
rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  return rmw_zenoh_common_context_fini(context, eclipse_zenoh_identifier);
}

/// DESTROY NODE ===============================================================
// Finalize a given node handle, reclaim the resources, and deallocate the node handle.
rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  return rmw_zenoh_common_destroy_node(node, eclipse_zenoh_identifier);
}

/// COPY OPTIONS ===============================================================
// Copy the given source init options to the destination init options.
rmw_ret_t
rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  return rmw_zenoh_common_init_options_copy(src, dst, eclipse_zenoh_identifier);
}

/// FINALIZE OPTIONS ===========================================================
// Finalize the given init_options. (Cleanup and deallocation.)
rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  return rmw_zenoh_common_init_options_fini(init_options, eclipse_zenoh_identifier);
}
