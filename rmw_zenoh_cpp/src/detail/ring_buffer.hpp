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

#ifndef DETAIL__RING_BUFFER_HPP_
#define DETAIL__RING_BUFFER_HPP_

#include <algorithm>
#include <stdexcept>
#include <vector>
#include <utility>

namespace rmw_zenoh_cpp
{
///=============================================================================
// FIFO circular buffer.
template<typename T>
class RingBuffer final
{
public:
  /// Constructor.
  explicit RingBuffer(std::size_t max_length)
  : max_length_(std::move(max_length)),
    front_idx_(0),
    back_idx_(0),
    size_(0)
  {
    // Reserve a constant size of memory for the data.
    data_.reserve(max_length_);
  }

  /// Returns the number of data entries presently stored.
  std::size_t size() const
  {
    return std::min(size_, max_length_);
  }

  /// Return the earliest entry from this buffer.
  /// Note: Throws an std::out_of_range exception if the buffer is empty.
  T pop()
  {
    if (this->size() == 0) {
      throw std::out_of_range("Buffer is empty");
    }
    auto ret = std::move(data_[front_idx_]);
    --size_;
    // If the buffer is empty after removing one element, we don't
    // need to increment front_idx_;
    if (this->size() != 0) {
      this->increment_idx(front_idx_);
    }

    return ret;
  }

  /// Add a new entry to the buffer.
  void append(T data)
  {
    this->increment_idx(back_idx_);
    data_[back_idx_] = std::move(data);
    ++size_;
    // If back_idx_ and front_idx_ are the same, increment front_idx_.
    if (front_idx_ == back_idx_) {
      this->increment_idx(front_idx_);
    }
  }

private:
  std::size_t max_length_;
  std::vector<T> data_;
  std::size_t front_idx_;
  std::size_t back_idx_;
  std::size_t size_;

  /// Safely increment the idx.
  void increment_idx(std::size_t & idx)
  {
    if (idx + 1 == max_length_) {
      idx = 0;
    } else {
      ++idx;
    }
  }
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__RING_BUFFER_HPP_
