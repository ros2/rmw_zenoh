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

#ifndef DETAIL__ZENOH_UTILS_HPP_
#define DETAIL__ZENOH_UTILS_HPP_

#include <zenoh.hxx>

#include <array>
#include <chrono>
#include <functional>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "rmw/types.h"

namespace rmw_zenoh_cpp
{
///=============================================================================
// A class to store the replies to service requests.
class ZenohReply final
{
public:
  ZenohReply(const zenoh::Reply & reply, std::chrono::nanoseconds::rep received_timestamp);

  ~ZenohReply() = default;

  const zenoh::Reply & get_sample() const;

  std::chrono::nanoseconds::rep get_received_timestamp() const;

private:
  std::optional<zenoh::Reply> reply_;
  std::chrono::nanoseconds::rep received_timestamp_;
};

// A class to store the queries made by clients.
///=============================================================================
class ZenohQuery final
{
public:
  ZenohQuery(const zenoh::Query & query, std::chrono::nanoseconds::rep received_timestamp);

  ~ZenohQuery() = default;

  const zenoh::Query & get_query() const;

  std::chrono::nanoseconds::rep get_received_timestamp() const;

private:
  zenoh::Query query_;
  std::chrono::nanoseconds::rep received_timestamp_;
};

int64_t get_system_time_in_ns();

class Payload
{
public:
  explicit Payload(const zenoh::Bytes & bytes);

  ~Payload() = default;

  const uint8_t * data() const;

  size_t size() const;

  bool empty() const;

private:
  struct Contiguous
  {
    zenoh::Slice slice;
    zenoh::Bytes bytes;
  };
  using NonContiguous = std::vector<uint8_t>;
  using Empty = std::nullptr_t;
  // Is `std::vector<uint8_t>` in case of a non-contiguous payload
  // and `zenoh::Slice` plus a `zenoh::Bytes` otherwise.
  std::variant<NonContiguous, Contiguous, Empty> bytes_;
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__ZENOH_UTILS_HPP_
