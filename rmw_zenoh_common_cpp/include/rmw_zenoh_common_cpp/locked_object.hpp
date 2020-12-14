// Copyright 2020 ADLINK, Inc.
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

#ifndef RMW_ZENOH_COMMON_CPP__LOCKED_OBJECT_HPP_
#define RMW_ZENOH_COMMON_CPP__LOCKED_OBJECT_HPP_

#include <mutex>

#include "rcpputils/thread_safety_annotations.hpp"

template<class T>
class LockedObject
{
private:
  mutable std::mutex mutex_;
  T object_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

public:
  /**
  * \return a reference to this object to lock.
  */
  std::mutex & getMutex() const RCPPUTILS_TSA_RETURN_CAPABILITY(mutex_)
  {
    return mutex_;
  }

  T & operator()()
  {
    return object_;
  }

  const T & operator()() const
  {
    return object_;
  }
};

#endif  // RMW_ZENOH_COMMON_CPP__LOCKED_OBJECT_HPP_
