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

#ifndef DETAIL__BUFFER_ENDPOINT_HELPERS_HPP_
#define DETAIL__BUFFER_ENDPOINT_HELPERS_HPP_

#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rmw/types.h"

#include "liveliness_utils.hpp"

namespace rmw_zenoh_cpp
{
namespace buffer_endpoint_helpers
{

/// Convert an `rmw_gid_t` to its lowercase hex string representation.
inline std::string gid_to_hex(const rmw_gid_t & gid)
{
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    out << std::setw(2) << static_cast<int>(gid.data[i]);
  }
  return out.str();
}

/// Convert a raw GID byte array to its lowercase hex string representation.
inline std::string gid_array_to_hex(
  const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & gid_array)
{
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (size_t i = 0; i < gid_array.size(); ++i) {
    out << std::setw(2) << static_cast<int>(gid_array[i]);
  }
  return out.str();
}

/// Build the shared CPU-group endpoint key for a base topic key expression.
///
/// All buffer-aware publishers and CPU-only subscribers on a topic share a
/// single Zenoh channel under this suffix.
inline std::string make_cpu_group_key(const std::string & base_key)
{
  return base_key + "/_buf_cpu";
}

/// Build the per-subscriber accelerated buffer key for a base topic key.
///
/// Each buffer-aware subscriber owns one accelerated channel keyed by its
/// GID. Every matching publisher writes to this key, so a subscriber only
/// needs a single accelerated Zenoh subscriber regardless of the number of
/// matched publishers.
inline std::string make_accelerated_key(
  const std::string & base_key,
  const std::string & sub_gid_hex)
{
  return base_key + "/_buf/" + sub_gid_hex;
}

/// Convenience helper: accelerated key from a publisher-side `rmw_gid_t`.
inline std::string make_accelerated_key(
  const std::string & base_key,
  const rmw_gid_t & sub_gid)
{
  return make_accelerated_key(base_key, gid_to_hex(sub_gid));
}

/// Stringify a liveliness EntityType for log messages.
inline const char * entity_type_to_string(liveliness::EntityType type)
{
  switch (type) {
    case liveliness::EntityType::Node:
      return "Node";
    case liveliness::EntityType::Publisher:
      return "Publisher";
    case liveliness::EntityType::Subscription:
      return "Subscription";
    case liveliness::EntityType::Service:
      return "Service";
    case liveliness::EntityType::Client:
      return "Client";
    default:
      return "Unknown";
  }
}

/// True iff the only advertised backend type is "cpu".
inline bool is_cpu_only_backend_types(const std::vector<std::string> & backend_types)
{
  return backend_types.size() == 1 && backend_types.front() == "cpu";
}

/// True iff the only advertised backend in the metadata map is "cpu".
inline bool is_cpu_only_backend_metadata(
  const std::unordered_map<std::string, std::string> & backend_metadata)
{
  return backend_metadata.size() == 1 && backend_metadata.count("cpu") == 1;
}

}  // namespace buffer_endpoint_helpers
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__BUFFER_ENDPOINT_HELPERS_HPP_
