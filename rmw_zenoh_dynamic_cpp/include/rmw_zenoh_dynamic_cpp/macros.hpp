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

#ifndef RMW_ZENOH_DYNAMIC_CPP__MACROS_HPP_
#define RMW_ZENOH_DYNAMIC_CPP__MACROS_HPP_

#include <limits>
#include <string>

#define SPECIALIZE_GENERIC_C_SEQUENCE(C_NAME, C_TYPE) \
  template<> \
  struct GenericCSequence<C_TYPE> \
  { \
    using type = rosidl_runtime_c__ ## C_NAME ## __Sequence; \
 \
    static void fini(type * array) { \
      rosidl_runtime_c__ ## C_NAME ## __Sequence__fini(array); \
    } \
 \
    static bool init(type * array, size_t size) { \
      return rosidl_runtime_c__ ## C_NAME ## __Sequence__init(array, size); \
    } \
  };

#endif  // RMW_ZENOH_DYNAMIC_CPP__MACROS_HPP_
