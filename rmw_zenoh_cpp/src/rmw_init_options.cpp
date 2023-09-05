// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "detail/rmw_init_options_impl.hpp"

#include "rmw/rmw.h"

//==============================================================================
/// Return a zero initialized init options structure.
// rmw_init_options_t
// rmw_get_zero_initialized_init_options(void)
// {
//   return RMW_RET_UNSUPPORTED;
// }

//==============================================================================
/// Initialize given init options with the default values and implementation specific values.
rmw_ret_t
rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator)
{
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Copy the given source init options to the destination init options.
rmw_ret_t
rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Finalize the given init options.
rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  return RMW_RET_UNSUPPORTED;
}
