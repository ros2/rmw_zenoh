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

#ifndef DETAIL__CDR_HPP_
#define DETAIL__CDR_HPP_

#include "fastcdr/Cdr.h"
#include "fastcdr/FastBuffer.h"

// A wrapper class to paper over the differences between Fast-CDR v1 and Fast-CDR v2
namespace rmw_zenoh_cpp
{
class Cdr final
{
public:
  explicit Cdr(eprosima::fastcdr::FastBuffer & fastbuffer);

  eprosima::fastcdr::Cdr & get_cdr();

  size_t get_serialized_data_length() const;

private:
  eprosima::fastcdr::Cdr cdr_;
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__CDR_HPP_
