// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_ZENOH_CPP__RMW_ZENOH_HPP_
#define RMW_ZENOH_CPP__RMW_ZENOH_HPP_

#include <memory>

#include <zenoh.hxx>

#include "rmw/types.h"
#include "rmw/visibility_control.h"

#ifdef __cplusplus

/// Get the Zenoh session associated with the given RMW context.
/// \param[in] context The RMW context.
/// \return A shared pointer to the Zenoh session, or nullptr if invalid.
RMW_PUBLIC
const std::shared_ptr<zenoh::Session>
rmw_zenoh_get_session(const rmw_context_t * context);

#endif  // __cplusplus

#endif  // RMW_ZENOH_CPP__RMW_ZENOH_HPP_
