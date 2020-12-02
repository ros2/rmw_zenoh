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

#ifndef TESTING_MACROS_HPP_
#define TESTING_MACROS_HPP_

#include <chrono>
#include <thread>

/// Retry until `timeout` expires, sleeping for `delay` in between attempts.
/*
 * \note Time is measured against OS provided steady clock.
 */
#define SLEEP_AND_RETRY_UNTIL(delay, timeout) for ( \
    auto loop_start_time = std::chrono::steady_clock::now(); \
    std::chrono::steady_clock::now() - loop_start_time < timeout; \
    std::this_thread::sleep_for(delay))

#endif  // TESTING_MACROS_HPP_
