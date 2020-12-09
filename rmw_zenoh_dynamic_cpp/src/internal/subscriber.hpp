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

#include <string>
#include <mutex>
#include <queue>
#include <memory>
#include <algorithm>
#include <functional>

#include <ecal/ecal.h>

#include <rmw/types.h>

#include "internal/qos.hpp"
#include "internal/typesupport.hpp"
#include "internal/event.hpp"

namespace eCAL
{
  namespace rmw
  {

    class Subscriber
    {
    public:
      struct Data
      {
        Data(char* data_, size_t size_) : data(data_), size(size_) {}
        char* data;
        size_t size;
      };

    private:
      std::unique_ptr<MessageTypeSupport> type_support_;
      eCAL::CSubscriber subscriber_;

      mutable std::mutex queue_mutex_;
      mutable std::mutex wait_set_mutex_;

      WaitSet* wait_set_ = nullptr;
      Event data_dropped_event_;

      std::queue<Data> data_;

      rmw_qos_profile_t ros_qos_profile_;

      void OnReceiveData(const char* /* topic */, const eCAL::SReceiveCallbackData* data)
      {
        auto latest_data = SaveData(data->buf, data->size);
        EnqueueData(latest_data, data->size);
        NotifyWaitSet();
      }

      void OnDataDropped(const char* /* topic_name */, const eCAL::SSubEventCallbackData* /* data */)
      {
        data_dropped_event_.Trigger();
      }

      char* SaveData(void* data, size_t data_size)
      {
        auto latest_data = new char[data_size];
        std::memcpy(latest_data, data, data_size);
        return latest_data;
      }

      void EnqueueData(char* data, size_t size)
      {
        std::lock_guard<std::mutex> queue_lock(queue_mutex_);
        data_.emplace(data, size);
      }

      void NotifyWaitSet()
      {
        auto wait_set = GetAttachedWaitSet();
        if (wait_set)
        {
          std::lock_guard<std::mutex> ws_condition_lock(wait_set->condition_mutex);
          wait_set->Trigger();
        }
      }

      Data PopData()
      {
        std::lock_guard<std::mutex> queue_lock(queue_mutex_);
        auto latest_data = data_.front();
        data_.pop();

        return latest_data;
      }

      void CleanupData()
      {
        while (HasData())
        {
          delete[] PopData().data;
        }
      }

      WaitSet* GetAttachedWaitSet() const
      {
        std::unique_lock<std::mutex> ws_lock(wait_set_mutex_);
        return wait_set_;
      }

    public:
      Subscriber(const std::string& topic_name, MessageTypeSupport* ts, const SubscriberQOS& qos)
        : type_support_(ts)
      {
        using namespace std::placeholders;

        ros_qos_profile_ = qos.rmw_qos;

        subscriber_ = eCAL::CSubscriber(qos.topic_name_prefix + topic_name,
                                        serialization_typename_prefix + type_support_->GetMessageName(),
                                        type_support_->GetTypeDescriptor());
        subscriber_.SetQOS(qos.ecal_qos);
        subscriber_.AddReceiveCallback(std::bind(&Subscriber::OnReceiveData, this, _1, _2));
        subscriber_.AddEventCallback(eCAL_Subscriber_Event::sub_event_dropped, std::bind(&Subscriber::OnDataDropped, this, _1, _2));
      }

      void TakeLatestData(void* data)
      {
        auto latest_data = PopData();
        type_support_->Deserialize(data, latest_data.data, latest_data.size);
        delete[] latest_data.data;
      }

      Data TakeLatestSerializedData()
      {
        return PopData();
      }

      bool HasData() const
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return !data_.empty();
      }

      std::string GetTopicName() const
      {
        return subscriber_.GetTopicName();
      }

      std::string GetTopicType() const
      {
        return subscriber_.GetTypeName();
      }

      const rmw_qos_profile_t& GetRosQOSProfile() const
      {
        return ros_qos_profile_;
      }

      Event& GetDataDroppedEventListener()
      {
        return data_dropped_event_;
      }

      void AttachWaitSet(WaitSet* wait_set)
      {
        std::unique_lock<std::mutex> lock(wait_set_mutex_);
        wait_set_ = wait_set;
      }

      void DetachWaitSet()
      {
        std::unique_lock<std::mutex> lock(wait_set_mutex_);
        wait_set_ = nullptr;
      }

      ~Subscriber()
      {
        CleanupData();
      }
    };

  } // namespace rmw
} // namespace eCAL
