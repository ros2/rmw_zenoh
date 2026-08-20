// Copyright 2026 ktyang512
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

#include <gtest/gtest.h>

#include "detail/rmw_wait.hpp"
#include "detail/rmw_wait_set_data.hpp"

TEST(TestRmwWait, preserves_trigger_set_during_condition_attachment)
{
  rmw_zenoh_cpp::rmw_wait_set_data_t wait_set_data{};

  // Model the relevant interleaving without scheduler-dependent sleeps: an attached entity sets
  // the flag while rmw_wait() is still scanning the remaining entities.
  wait_set_data.triggered = true;

  // Finishing the scan must not erase the notification before rmw_wait() evaluates its predicate.
  EXPECT_FALSE(rmw_zenoh_cpp::detail::condition_attachment_scan_complete(&wait_set_data));
  EXPECT_TRUE(wait_set_data.triggered);
}
