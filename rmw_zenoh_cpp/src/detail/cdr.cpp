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

#include "fastcdr/Cdr.h"
#include "fastcdr/FastBuffer.h"
#include "fastcdr/config.h"

#include "cdr.hpp"

namespace rmw_zenoh_cpp
{
Cdr::Cdr(eprosima::fastcdr::FastBuffer & fastbuffer)
#if FASTCDR_VERSION_MAJOR == 1
: cdr_(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR)
#else
: cdr_(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::CdrVersion::DDS_CDR)
#endif
{
}

size_t Cdr::get_serialized_data_length() const
{
#if FASTCDR_VERSION_MAJOR == 1
  return cdr_.getSerializedDataLength();
#else
  return cdr_.get_serialized_data_length();
#endif
}

eprosima::fastcdr::Cdr & Cdr::get_cdr()
{
  return cdr_;
}
}  // namespace rmw_zenoh_cpp
