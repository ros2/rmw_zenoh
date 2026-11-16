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

#include <string>
#include <utility>
#include <vector>

#include "detail/endpoint_info.hpp"

using rmw_zenoh_cpp::EndpointInfoStorage;

namespace
{
EndpointInfoStorage make_storage(const std::string & name)
{
  EndpointInfoStorage storage;
  storage.node_name = name;
  storage.node_namespace = "/ns";
  storage.topic_type = "test_msgs::msg::dds_::UnboundedSequences_";
  storage.bind_strings();
  return storage;
}

void expect_bound(const EndpointInfoStorage & storage, const std::string & name)
{
  EXPECT_EQ(storage.info.node_name, storage.node_name.c_str());
  EXPECT_EQ(storage.info.node_namespace, storage.node_namespace.c_str());
  EXPECT_EQ(storage.info.topic_type, storage.topic_type.c_str());
  EXPECT_STREQ(storage.info.node_name, name.c_str());
}
}  // namespace

TEST(TestEndpointInfo, CopyAndMoveRebindStrings)
{
  // Long enough to defeat the small string optimization.
  const std::string name = "a_node_name_longer_than_the_small_string_buffer";
  EndpointInfoStorage original = make_storage(name);

  EndpointInfoStorage copy(original);
  expect_bound(copy, name);

  EndpointInfoStorage copy_assigned;
  copy_assigned = original;
  expect_bound(copy_assigned, name);

  EndpointInfoStorage moved(std::move(original));
  expect_bound(moved, name);

  EndpointInfoStorage move_assigned;
  move_assigned = std::move(moved);
  expect_bound(move_assigned, name);
}

TEST(TestEndpointInfo, VectorGrowthRebindStrings)
{
  std::vector<EndpointInfoStorage> storages;
  for (int i = 0; i < 32; ++i) {
    storages.push_back(make_storage("node_" + std::to_string(i)));
  }
  for (int i = 0; i < 32; ++i) {
    expect_bound(storages[i], "node_" + std::to_string(i));
  }
}
