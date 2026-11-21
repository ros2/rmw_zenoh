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

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

#include "detail/liveliness_utils.hpp"

#include "rmw/qos_profiles.h"
#include "rmw/types.h"

#if defined(_MSC_VER)
#define NOINLINE __declspec(noinline)
#else
#define NOINLINE __attribute__((noinline))
#endif

using rmw_zenoh_cpp::liveliness::keyexpr_to_qos;
using rmw_zenoh_cpp::liveliness::qos_to_keyexpr;

namespace
{
// Leave a recognizable pattern in the stack area that the next call reuses, so that a field
// keyexpr_to_qos() forgets to set shows up as that pattern instead of a lucky zero.
NOINLINE void dirty_stack(unsigned char pattern)
{
  volatile unsigned char buffer[16384];
  for (auto & byte : buffer) {
    byte = pattern;
  }
}

// Read the bool's storage without converting it: any value other than 0 or 1 is not a valid bool.
uint8_t raw_byte(const bool & value)
{
  uint8_t raw;
  std::memcpy(&raw, &value, sizeof(raw));
  return raw;
}
}  // namespace

TEST(TestLivelinessUtils, KeyexprToQosRoundTrip)
{
  rmw_qos_profile_t offered = rmw_qos_profile_default;
  offered.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  offered.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  offered.depth = 42;

  const std::optional<rmw_qos_profile_t> decoded = keyexpr_to_qos(qos_to_keyexpr(offered));
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->history, offered.history);
  EXPECT_EQ(decoded->depth, offered.depth);
  EXPECT_EQ(decoded->reliability, offered.reliability);
  EXPECT_EQ(decoded->durability, offered.durability);
  EXPECT_EQ(decoded->liveliness, offered.liveliness);
}

TEST(TestLivelinessUtils, KeyexprToQosInitializesAvoidRosNamespaceConventions)
{
  // avoid_ros_namespace_conventions is not encoded in the keyexpr, so the decoded profile
  // must report false rather than whatever was on the stack.
  const std::string keyexpr = qos_to_keyexpr(rmw_qos_profile_sensor_data);
  for (unsigned char pattern : {0x01, 0xA5, 0xFF}) {
    dirty_stack(pattern);
    const std::optional<rmw_qos_profile_t> decoded = keyexpr_to_qos(keyexpr);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(raw_byte(decoded->avoid_ros_namespace_conventions), 0u) <<
      "stack pattern 0x" << std::hex << static_cast<int>(pattern);
  }
}
