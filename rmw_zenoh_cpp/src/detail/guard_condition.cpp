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

#include "guard_condition.hpp"

///==============================================================================
GuardCondition::GuardCondition()
: has_triggered_(false),
  condition_mutex_(nullptr),
  condition_variable_(nullptr) {}

///==============================================================================
void GuardCondition::trigger()
{
  std::lock_guard<std::mutex> lock(internal_mutex_);

  if (condition_mutex_ != nullptr) {
    std::unique_lock<std::mutex> clock(*condition_mutex_);
    // the change to hasTriggered_ needs to be mutually exclusive with
    // rmw_wait() which checks hasTriggered() and decides if wait() needs to
    // be called
    has_triggered_ = true;
    clock.unlock();
    condition_variable_->notify_one();
  } else {
    has_triggered_ = true;
  }
}

///==============================================================================
void GuardCondition::attach_condition(
  std::mutex * condition_mutex,
  std::condition_variable * condition_variable)
{
  std::lock_guard<std::mutex> lock(internal_mutex_);
  condition_mutex_ = condition_mutex;
  condition_variable_ = condition_variable;
}

///==============================================================================
void GuardCondition::detach_condition()
{
  std::lock_guard<std::mutex> lock(internal_mutex_);
  condition_mutex_ = nullptr;
  condition_variable_ = nullptr;
}

///==============================================================================
bool GuardCondition::has_triggered() const
{
  return has_triggered_;
}

///==============================================================================
bool GuardCondition::get_has_triggered()
{
  bool ret = has_triggered_;
  has_triggered_ = false;
  return ret;
}
