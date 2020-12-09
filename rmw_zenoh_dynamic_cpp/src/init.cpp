// Copyright 2020 Continental AG
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

#include <rmw/init.h>

#include <rmw/rmw.h>
#include <rmw/error_handling.h>
#include <rmw/impl/cpp/macros.hpp>

#include <ecal/ecal.h>

#include "internal/common.hpp"

rmw_context_t rmw_get_zero_initialized_context(void)
{
  return {};
}

rmw_ret_t rmw_init(const rmw_init_options_t* options, rmw_context_t* context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  int status = eCAL::Initialize(0, nullptr, nullptr, eCAL::Init::Default | eCAL::Init::Monitoring);
  if (status == -1)
    return RMW_RET_ERROR;

  eCAL::Util::EnableLoopback(true);
  eCAL::Process::SetState(eCAL_Process_eSeverity::proc_sev_healthy,
    eCAL_Process_eSeverity_Level::proc_sev_level1,
    "Initializing");
  eCAL::Process::SleepMS(1000);
  eCAL::Process::SetState(eCAL_Process_eSeverity::proc_sev_healthy,
    eCAL_Process_eSeverity_Level::proc_sev_level1,
    "Running");

  context->instance_id = options->instance_id;
  context->impl = nullptr;
  context->implementation_identifier = options->implementation_identifier;

  return RMW_RET_OK;
}

rmw_ret_t rmw_shutdown(rmw_context_t* context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(context);

  int status = eCAL::Finalize();
  if (status == -1)
    return RMW_RET_ERROR;

  return RMW_RET_OK;
}

rmw_ret_t rmw_context_fini(rmw_context_t* context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(context);

  *context = rmw_get_zero_initialized_context();

  return RMW_RET_OK;
}
