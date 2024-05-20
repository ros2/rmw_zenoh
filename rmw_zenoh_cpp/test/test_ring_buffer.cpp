// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <iostream>

#include "../src/detail/ring_buffer.hpp"

///=============================================================================
TEST(TestRingBuffer, check_size) {
  const std::size_t max_size = 5;
  rmw_zenoh_cpp::RingBuffer<std::size_t> buffer{max_size};
  EXPECT_EQ(buffer.size(), 0);
  // Append numbers to the buffer upto its max_size.
  for (std::size_t i = 0; i < max_size; ++i) {
    buffer.append(i);
    EXPECT_EQ(buffer.size(), i + 1);
  }
  // Then check that elements are as we expect.
  for (std::size_t i = 0; i < max_size; ++i) {
    EXPECT_EQ(buffer.pop(), i);
  }
  // Then the buffer should be empty.
  EXPECT_EQ(buffer.size(), 0);
}

///=============================================================================
TEST(TestRingBuffer, check_overflow) {
  const std::size_t max_size = 5;
  rmw_zenoh_cpp::RingBuffer<std::size_t> buffer{max_size};
  EXPECT_EQ(buffer.size(), 0);
  // Append numbers past the max_size
  const std::size_t overflow = 7;
  const std::size_t max_element = max_size + overflow;
  for (std::size_t i = 0; i < max_element; ++i) {
    buffer.append(i);
  }
  // The size of the buffer should be max_size.
  EXPECT_EQ(buffer.size(), max_size);

  // Check elements.
  for (std::size_t i = 0; i < max_size; ++i) {
    EXPECT_EQ(buffer.pop(), (max_element - max_size + i));
  }
}

///=============================================================================
TEST(TestRingBuffer, check_pop_when_empty) {
  const std::size_t max_size = 5;
  rmw_zenoh_cpp::RingBuffer<std::size_t> buffer{max_size};
  try {
    buffer.pop();
  } catch (const std::out_of_range & e) {
    EXPECT_EQ(e.what(), std::string("Buffer is empty"));
  }
}
