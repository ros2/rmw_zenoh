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
#include <memory>

#include <ecal/ecal.h>

#include <rmw/types.h>

#include "internal/qos.hpp"
#include "internal/typesupport.hpp"
#include "internal/event.hpp"

namespace eCAL
{
  namespace rmw
  {
    class Publisher
    {
      std::unique_ptr<MessageTypeSupport> type_support_;
      eCAL::CPublisher                    publisher_;
      rmw_qos_profile_t                   ros_qos_profile_;
      Event                               data_dropped_event_;

      void OnDataDropped(const char* /* topic_name */, const eCAL::SPubEventCallbackData* /* data */)
      {
        data_dropped_event_.Trigger();
      }

    public:
      Publisher(const std::string& topic_name, MessageTypeSupport* ts, const PublisherQOS& qos)
        : type_support_(ts)
      {
        using namespace std::placeholders;

        ros_qos_profile_ = qos.rmw_qos;

        publisher_ = eCAL::CPublisher(qos.topic_name_prefix + topic_name,
                                      serialization_typename_prefix + type_support_->GetMessageName(),
                                      type_support_->GetTypeDescriptor());
        publisher_.SetQOS(qos.ecal_qos);
        publisher_.AddEventCallback(eCAL_Publisher_Event::pub_event_dropped, std::bind(&Publisher::OnDataDropped, this, _1, _2));
      }

      void Publish(const void* data)
      {
        auto serialized_data = type_support_->Serialize(data);
        publisher_.Send(serialized_data.data(), serialized_data.size());
      }

      void PublishRaw(const void* data, const size_t data_size)
      {
        publisher_.Send(data, data_size);
      }

      const rmw_qos_profile_t& GetRosQOSProfile() const
      {
        return ros_qos_profile_;
      }

      std::string GetTopicName() const
      {
        return publisher_.GetTopicName();
      }

      std::string GetTopicType() const
      {
        return publisher_.GetTypeName();
      }

      Event& GetDataDroppedEventListener()
      {
        return data_dropped_event_;
      }

      void UnregisterEvent(Event* /*event*/)
      {
        //
      }
    };

  } // namespace rmw
} // namespace eCAL
