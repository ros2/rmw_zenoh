// Copyright 2020 ADLINK, Inc.
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
{
  (void)session;
}
