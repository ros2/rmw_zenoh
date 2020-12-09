// Copyright 2020 Continental AG
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

#pragma once

#include <iterator>
#include <type_traits>
#include <tuple>

#include <rmw/types.h>
#include <rmw/names_and_types.h>

namespace eCAL
{
  namespace rmw
  {
    namespace RosArray
    {
      template <typename ROS_ARRAY_T>
      class iterator
      {
        using iterator_category = std::random_access_iterator_tag;
        using value_type = typename std::remove_pointer<decltype(ROS_ARRAY_T::data)>::type;
        using difference_type = std::ptrdiff_t;
        using reference = value_type&;
        using pointer = value_type*;

        pointer current_;

      public:
        iterator(pointer start) : current_{ start } {}
        // TODO: PVS issue V690 (https://www.viva64.com/en/w/v690/print/)
        // should be .. ?
        // iterator(const iterator& rhs) = { current_ = rhs.current_; }
        iterator(const iterator&) = default;
        ~iterator() = default;

        iterator& operator=(const iterator& rhs)
        {
          current_ = rhs.current_;
          return *this;
        }

        iterator& operator++()
        {
          current_++;
          return *this;
        }

        iterator operator++(int)
        {
          auto old = *this;
          current_++;
          return old;
        }

        iterator& operator--()
        {
          current_--;
          return *this;
        }

        iterator operator--(int)
        {
          auto old = *this;
          current_--;
          return old;
        }

        bool operator<(const iterator& other)
        {
          return current_ < other.current_;
        }

        bool operator>(const iterator& other)
        {
          return current_ > other.current_;
        }

        bool operator<=(const iterator& other)
        {
          return current_ <= other.current_;
        }

        bool operator>=(const iterator& other)
        {
          return current_ >= other.current_;
        }

        iterator operator+(difference_type offset)
        {
          return iterator{ current_ + offset };
        }

        iterator& operator+=(difference_type offset)
        {
          current_ += offset;
          return *this;
        }

        iterator& operator-=(difference_type offset)
        {
          current_ -= offset;
          return *this;
        }

        iterator operator-(difference_type offset)
        {
          return iterator{ current_ - offset };
        }

        difference_type operator-(iterator other)
        {
          return reinterpret_cast<difference_type>(other.current_ - current_);
        }

        bool operator==(const iterator& other)
        {
          return current_ == other.current_;
        }

        bool operator!=(const iterator& other)
        {
          return !(*this == other);
        }

        reference operator*() const
        {
          return *current_;
        }

        pointer operator->() const
        {
          return current_;
        }

        reference operator[](difference_type index) const
        {
          return *(current_[index]);
        }
      };

      template <>
      class iterator<rmw_names_and_types_t>
      {
        using iterator_category = std::random_access_iterator_tag;
        using value_type = std::tuple<char*&, rcutils_string_array_t&>;
        using difference_type = decltype(std::declval<decltype(rmw_names_and_types_t::names)>().size);
        using reference = value_type&;
        using pointer = value_type*;

        rmw_names_and_types_t& arr_;
        difference_type index_;

      public:
        iterator(rmw_names_and_types_t& arr, difference_type index) : arr_{ arr }, index_{ index } {}
        iterator(const iterator&) = default;
        ~iterator() = default;

        iterator& operator=(const iterator& rhs)
        {
          index_ = rhs.index_;
          return *this;
        }

        iterator& operator++()
        {
          index_++;
          return *this;
        }

        iterator operator++(int)
        {
          auto old = *this;
          index_++;
          return old;
        }

        iterator& operator--()
        {
          index_--;
          return *this;
        }

        iterator operator--(int)
        {
          auto old = *this;
          index_--;
          return old;
        }

        bool operator<(const iterator& other)
        {
          return index_ < other.index_;
        }

        bool operator>(const iterator& other)
        {
          return index_ > other.index_;
        }

        bool operator<=(const iterator& other)
        {
          return index_ <= other.index_;
        }

        bool operator>=(const iterator& other)
        {
          return index_ >= other.index_;
        }

        iterator operator+(difference_type offset)
        {
          return iterator{ arr_, index_ + offset };
        }

        iterator& operator+=(difference_type offset)
        {
          index_ += offset;
          return *this;
        }

        iterator& operator-=(difference_type offset)
        {
          index_ -= offset;
          return *this;
        }

        iterator operator-(difference_type offset)
        {
          return iterator{ arr_, index_ - offset };
        }

        difference_type operator-(iterator other)
        {
          return index_ - other.index_;
        }

        bool operator==(const iterator& other)
        {
          return index_ == other.index_;
        }

        bool operator!=(const iterator& other)
        {
          return !(*this == other);
        }

        value_type operator*() const
        {
          return std::forward_as_tuple(arr_.names.data[index_], arr_.types[index_]);
        }

        value_type operator->() const
        {
          return std::forward_as_tuple(arr_.names.data[index_], arr_.types[index_]);
        }

        value_type operator[](difference_type index) const
        {
          return std::forward_as_tuple(arr_.names.data[index], arr_.types[index]);
        }
      };

      template <typename ROS_ARRAY_T>
      inline iterator<ROS_ARRAY_T> Begin(ROS_ARRAY_T& array)
      {
        return iterator<ROS_ARRAY_T>{array.data};
      }

      template <typename ROS_ARRAY_T>
      inline iterator<ROS_ARRAY_T> End(ROS_ARRAY_T& array)
      {
        return iterator<ROS_ARRAY_T>{array.data + array.size};
      }

      template <>
      inline iterator<rmw_names_and_types_t> Begin(rmw_names_and_types_t& array)
      {
        return iterator<rmw_names_and_types_t>{array, 0};
      }

      template <>
      inline iterator<rmw_names_and_types_t> End(rmw_names_and_types_t& array)
      {
        return iterator<rmw_names_and_types_t>{array, array.names.size - 1};
      }

    } // namespace RosArrayIterator
  } // namespace rmw
} // namespace eCAL
