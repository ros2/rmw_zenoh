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
  condition_variable_(nullptr)
{
}

///==============================================================================
void GuardCondition::trigger()
{
  std::lock_guard<std::mutex> lock(internal_mutex_);

  // the change to hasTriggered_ needs to be mutually exclusive with
  // rmw_wait() which checks hasTriggered() and decides if wait() needs to
  // be called
  has_triggered_ = true;

  if (condition_variable_ != nullptr) {
    condition_variable_->notify_one();
  }
}

///==============================================================================
void GuardCondition::attach_condition(std::condition_variable * condition_variable)
{
  std::lock_guard<std::mutex> lock(internal_mutex_);
  condition_variable_ = condition_variable;
}

///==============================================================================
void GuardCondition::detach_condition()
{
  std::lock_guard<std::mutex> lock(internal_mutex_);
  condition_variable_ = nullptr;
}

///==============================================================================
bool GuardCondition::has_triggered() const
{
  std::lock_guard<std::mutex> lock(internal_mutex_);
  return has_triggered_;
}

///==============================================================================
void GuardCondition::reset_trigger()
{
  std::lock_guard<std::mutex> lock(internal_mutex_);
  has_triggered_ = false;
}
