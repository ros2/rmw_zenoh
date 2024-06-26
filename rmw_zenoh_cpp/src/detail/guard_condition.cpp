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

#include <mutex>

#include "guard_condition.hpp"
#include "rmw_wait_set_data.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
GuardCondition::GuardCondition()
: has_triggered_(false),
  wait_set_data_(nullptr)
{
}

///=============================================================================
void GuardCondition::trigger()
{
  std::lock_guard<std::mutex> lock(internal_mutex_);

  // the change to hasTriggered_ needs to be mutually exclusive with
  // rmw_wait() which checks hasTriggered() and decides if wait() needs to
  // be called
  has_triggered_ = true;

  if (wait_set_data_ != nullptr) {
    std::lock_guard<std::mutex> wait_set_lock(wait_set_data_->condition_mutex);
    wait_set_data_->triggered = true;
    wait_set_data_->condition_variable.notify_one();
  }
}

///=============================================================================
bool GuardCondition::check_and_attach_condition_if_not(rmw_wait_set_data_t * wait_set_data)
{
  std::lock_guard<std::mutex> lock(internal_mutex_);
  if (has_triggered_) {
    return true;
  }

  wait_set_data_ = wait_set_data;

  return false;
}

///=============================================================================
bool GuardCondition::detach_condition_and_is_trigger_set()
{
  std::lock_guard<std::mutex> lock(internal_mutex_);
  wait_set_data_ = nullptr;

  bool ret = has_triggered_;

  has_triggered_ = false;

  return ret;
}

}  // namespace rmw_zenoh_cpp
