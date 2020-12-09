// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef ALLOCATOR_TESTING_UTILS_H_
#define ALLOCATOR_TESTING_UTILS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>

#include "rcutils/allocator.h"

typedef struct __failing_allocator_state
{
  bool is_failing;
} __failing_allocator_state;

void *
failing_malloc(size_t size, void * state)
{
  if (((__failing_allocator_state *)state)->is_failing) {
    return nullptr;
  }
  return rcutils_get_default_allocator().allocate(size, rcutils_get_default_allocator().state);
}

void *
failing_realloc(void * pointer, size_t size, void * state)
{
  if (((__failing_allocator_state *)state)->is_failing) {
    return nullptr;
  }
  return rcutils_get_default_allocator().reallocate(
    pointer, size, rcutils_get_default_allocator().state);
}

void
failing_free(void * pointer, void * state)
{
  if (((__failing_allocator_state *)state)->is_failing) {
    return;
  }
  rcutils_get_default_allocator().deallocate(pointer, rcutils_get_default_allocator().state);
}

void *
failing_calloc(size_t number_of_elements, size_t size_of_element, void * state)
{
  if (((__failing_allocator_state *)state)->is_failing) {
    return nullptr;
  }
  return rcutils_get_default_allocator().zero_allocate(
    number_of_elements, size_of_element, rcutils_get_default_allocator().state);
}

static inline rcutils_allocator_t
get_failing_allocator(void)
{
  static __failing_allocator_state state;
  state.is_failing = true;
  auto failing_allocator = rcutils_get_default_allocator();
  failing_allocator.allocate = failing_malloc;
  failing_allocator.deallocate = failing_free;
  failing_allocator.reallocate = failing_realloc;
  failing_allocator.zero_allocate = failing_calloc;
  failing_allocator.state = &state;
  return failing_allocator;
}

static inline void
set_failing_allocator_is_failing(rcutils_allocator_t & failing_allocator, bool state)
{
  ((__failing_allocator_state *)failing_allocator.state)->is_failing = state;
}

#ifdef __cplusplus
}
#endif

#endif  // ALLOCATOR_TESTING_UTILS_H_
