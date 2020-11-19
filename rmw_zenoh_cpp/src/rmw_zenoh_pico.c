#include "rmw/rmw.h"
#include "rmw_zenoh_cpp/rmw_init_options_impl.hpp"
#include "rmw_zenoh_cpp/zenoh-net-interface.h"

#include "rmw/error_handling.h"

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
