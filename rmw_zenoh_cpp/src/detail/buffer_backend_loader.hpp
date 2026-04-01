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

#ifndef DETAIL__BUFFER_BACKEND_LOADER_HPP_
#define DETAIL__BUFFER_BACKEND_LOADER_HPP_

#include "buffer_backend_context.hpp"

namespace rmw_zenoh_cpp
{

/// Load buffer backend plugins and register them with FastCDR serialization.
/// Populates an RMW-context-local serialization map so multiple contexts in the
/// same process do not share mutable global descriptor state.
void initialize_buffer_backends(BufferBackendContext & context);

/// Clear context-local serialization maps.
void shutdown_buffer_backends(BufferBackendContext & context);

}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__BUFFER_BACKEND_LOADER_HPP_
