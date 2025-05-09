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

#include <array>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include <zenoh.hxx>

#include "rmw/types.h"

#include "attachment_helpers.hpp"
#include "liveliness_utils.hpp"

namespace rmw_zenoh_cpp
{

AttachmentData::AttachmentData(
  const int64_t sequence_number,
  const int64_t source_timestamp,
  const std::array<uint8_t, RMW_GID_STORAGE_SIZE> source_gid)
: sequence_number_(sequence_number),
  source_timestamp_(source_timestamp),
  source_gid_(source_gid)
{
}

AttachmentData::AttachmentData(AttachmentData && data)
{
  sequence_number_ = std::move(data.sequence_number_);
  source_timestamp_ = std::move(data.source_timestamp_);
  source_gid_ = data.source_gid_;
}

///=============================================================================
int64_t AttachmentData::sequence_number() const
{
  return sequence_number_;
}

///=============================================================================
int64_t AttachmentData::source_timestamp() const
{
  return source_timestamp_;
}

///=============================================================================
std::array<uint8_t, RMW_GID_STORAGE_SIZE> AttachmentData::copy_gid() const
{
  return source_gid_;
}

zenoh::Bytes AttachmentData::serialize_to_zbytes()
{
  auto serializer = zenoh::ext::Serializer();
  serializer.serialize(this->sequence_number_);
  serializer.serialize(this->source_timestamp_);
  serializer.serialize(this->source_gid_);
  return std::move(serializer).finish();
}

AttachmentData::AttachmentData(const zenoh::Bytes & bytes)
{
  zenoh::ext::Deserializer deserializer(bytes);
  this->sequence_number_ = deserializer.deserialize<int64_t>();
  this->source_timestamp_ = deserializer.deserialize<int64_t>();
  this->source_gid_ = deserializer.deserialize<std::array<uint8_t, RMW_GID_STORAGE_SIZE>>();
}
}  // namespace rmw_zenoh_cpp
