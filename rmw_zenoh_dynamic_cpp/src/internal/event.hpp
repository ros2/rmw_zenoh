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

#include <atomic>
#include <mutex>

#include <ecal/ecal.h>

#include "internal/wait_set.hpp"

namespace eCAL
{
  namespace rmw
  {

    class Event
    {
      WaitSet*            wait_set_ = nullptr;
      std::atomic_uint8_t triggered_count_;
      std::mutex          internal_mutex_;

    public:
      Event() : triggered_count_{ 0 }
      {
      }

      void Trigger()
      {
        std::lock_guard<std::mutex> lock(internal_mutex_);
        if (wait_set_ != nullptr)
        {
          std::unique_lock<std::mutex> clock(wait_set_->condition_mutex);

          triggered_count_++;
          clock.unlock();
          wait_set_->Trigger();
        }
        else
        {
          triggered_count_++;
        }
      }

      bool Triggered() const
      {
        return triggered_count_ > 0;
      }

      bool TakeTriggered()
      {
        if (triggered_count_ == 0)
        {
          return false;
        }
        triggered_count_--;
        return true;
      }

      void AttachWaitSet(WaitSet* wait_set)
      {
        std::lock_guard<std::mutex> lock(internal_mutex_);
        wait_set_ = wait_set;
      }

      void DetachWaitSet()
      {
        std::lock_guard<std::mutex> lock(internal_mutex_);
        wait_set_ = nullptr;
      }
    };

  } // namespace rmw
} // namespace eCAL
