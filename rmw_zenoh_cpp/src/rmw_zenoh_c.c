#include "rmw/rmw.h"
#include "rmw_zenoh_cpp/rmw_init_options_impl.hpp"
#include "rmw_zenoh_cpp/zenoh-net-interface.h"

zn_properties_t * configure_connection_mode(rmw_context_t * context)
{
  if (strcmp(context->options.impl->mode, "CLIENT") == 0) {
    return zn_config_client(context->options.impl->session_locator);
  } else {
    return zn_config_peer();
  }
}

void configure_session(zn_session_t * session)
{}
