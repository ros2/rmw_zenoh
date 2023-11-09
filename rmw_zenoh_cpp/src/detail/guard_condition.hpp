// Copyright 2023 Open Source Robotics Foundation, Inc.
// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef DETAIL__GUARD_CONDITION_HPP_
#define DETAIL__GUARD_CONDITION_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>

///==============================================================================
class GuardCondition final
{
public:
  GuardCondition();

  // Sets has_triggered_ to true and calls notify_one() on condition_variable_ if set.
  void trigger();

  void attach_condition(std::condition_variable * condition_variable);

  void detach_condition();

  bool has_triggered() const;

  // Resets has_triggered_ to false.
  void reset_trigger();

private:
  mutable std::mutex internal_mutex_;
  std::atomic_bool has_triggered_;
  std::condition_variable * condition_variable_;
};

#endif  // DETAIL__GUARD_CONDITION_HPP_
