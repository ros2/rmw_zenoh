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

#ifndef RMW_ZENOH_DYNAMIC_CPP__INIT_RMW_CONTEXT_IMPL_HPP_
#define RMW_ZENOH_DYNAMIC_CPP__INIT_RMW_CONTEXT_IMPL_HPP_

#include "rmw/init.h"
#include "rmw/types.h"

namespace rmw_zenoh_dynamic_cpp
{

/// Increment `rmw_context_impl_t` reference count, initializing it if necessary.
/**
 * Should be called when creating a node, and before using `context->impl`.
 */
rmw_ret_t
increment_context_impl_ref_count(rmw_context_t * context);

}  // namespace rmw_zenoh_dynamic_cpp

#endif  // RMW_ZENOH_DYNAMIC_CPP__INIT_RMW_CONTEXT_IMPL_HPP_
