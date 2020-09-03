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

#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <chrono>

namespace
{

// Quantities below were empirically adjusted to match the runtime
// behavior of the following implementations:
// - rmw_fastrtps_cpp
// - rmw_fastrtps_dynamic_cpp
// - rmw_connext_cpp
// - rmw_cyclonedds_cpp
// within ci.ros2.org instances.

std::chrono::milliseconds rmw_intraprocess_discovery_delay{100};

}  // namespace

#endif  // CONFIG_HPP_
