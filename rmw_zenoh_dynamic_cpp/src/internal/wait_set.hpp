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

#include <mutex>
#include <chrono>
#include <condition_variable>

namespace eCAL
{
  namespace rmw
  {

    class WaitSet
    {
    public:
      std::condition_variable condition;
      std::mutex condition_mutex;

      template <class _Rep, class _Period>
      bool Wait(std::unique_lock<std::mutex>& lock, const std::chrono::duration<_Rep, _Period>& duration)
      {
        return condition.wait_for(lock, duration) == std::cv_status::no_timeout;
      }

      void Wait(std::unique_lock<std::mutex>& lock)
      {
        return condition.wait(lock);
      }

      void Trigger()
      {
        condition.notify_all();
      }
    };

  } // namespace rmw
} // namespace eCAL
