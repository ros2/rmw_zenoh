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
#include <queue>
#include <utility>
#include <atomic>
#include <mutex>
#include <functional>
#include <unordered_map>
#include <condition_variable>

#include <ecal/ecal.h>

#include "internal/common.hpp"
#include "internal/qos.hpp"

namespace eCAL
{
  namespace rmw
  {

    class Service
    {
      class Request
      {
        const std::string&      request_;
        const sequence_number_t request_id_;
        std::string             response_;
        std::mutex              waiter_mutex_;
        std::condition_variable waiter_;

        static sequence_number_t GenerateRequestId()
        {
          static std::atomic<sequence_number_t> id{ 0 };
          return id++;
        }

      public:
        Request(const std::string& request) : request_(request),
          request_id_(GenerateRequestId())
        {
        }

        size_t GetRequestSize() const
        {
          return eCAL::rmw::GetSequenceDataSize(request_);
        }

        const void* GetRequest() const
        {
          return request_.data();
        }

        sequence_number_t GetRequestId() const
        {
          return request_id_;
        }

        sequence_number_t GetSequenceNumber() const
        {
          return eCAL::rmw::GetSequenceNumber(request_);
        }

        void WaitForResponse()
        {
          std::unique_lock<std::mutex> lock(waiter_mutex_);
          waiter_.wait(lock);
        }

        void ConsumeResponse(std::string&& response)
        {
          response_ = std::move(response);
          waiter_.notify_one();
        }

        const std::string& GetResponse() const
        {
          return response_;
        }
      };

      std::string name_;
      std::unique_ptr<ServiceTypeSupport> type_support_;
      eCAL::CServiceServer service_;

      WaitSet* wait_set_ = nullptr;

      mutable std::mutex wait_set_mutex_;
      mutable std::mutex pending_requests_mutex_;
      mutable std::mutex current_requests_mutex_;

      std::queue<std::reference_wrapper<Request>> pending_requests_;
      std::unordered_map<sequence_number_t, std::reference_wrapper<Request>> current_requests_;

      int OnRequest(const std::string& /* method */,
        const std::string& /* req_type */, const std::string& /* resp_type */,
        const std::string& request, std::string& response)
      {
        Request req(request);
        EnqueueRequest(req);
        NotifyWaitSet();
        req.WaitForResponse();

        response = req.GetResponse();
        auto sequence_number = req.GetSequenceNumber();
        auto sequence_number_bytes = reinterpret_cast<char*>(&sequence_number);
        response.append(sequence_number_bytes, sizeof(sequence_number_t));

        return 1;
      }

      int OnPingRequest(const std::string& /* method */,
                        const std::string& /* req_type */, const std::string& /* resp_type */,
                        const std::string& /* request */, std::string& /* response */)
      {
        return 1;
      }

      void EnqueueRequest(Request& request)
      {
        std::lock_guard<std::mutex> lock(pending_requests_mutex_);
        pending_requests_.push(request);
      }

      Request& PopNextRequest()
      {
        std::lock_guard<std::mutex> lock(pending_requests_mutex_);
        auto& request = pending_requests_.front();
        pending_requests_.pop();
        return request;
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

      WaitSet* GetAttachedWaitSet() const
      {
        std::unique_lock<std::mutex> ws_lock(wait_set_mutex_);
        return wait_set_;
      }

    public:
      Service(const std::string& name, ServiceTypeSupport* type_support, const ServiceQOS& qos) : name_(name),
              type_support_(type_support),
              service_(qos.service_name_prefix + name)
      {
        using namespace std::placeholders;

        service_.AddMethodCallback(type_support->GetServiceSimpleName(),
          type_support_->GetRequestMessageName(),
          type_support_->GetResponseMessageName(),
          std::bind(&Service::OnRequest, this, _1, _2, _3, _4, _5));

        service_.AddMethodCallback("_Ping" + type_support->GetServiceSimpleName(), "Empty", "Empty",
          std::bind(&Service::OnPingRequest, this, _1, _2, _3, _4, _5));
      }

      bool HasRequest() const
      {
        std::lock_guard<std::mutex> lock(pending_requests_mutex_);
        return !pending_requests_.empty();
      }

      sequence_number_t TakeRequest(void* data)
      {
        auto& request = PopNextRequest();
        current_requests_.insert(std::make_pair(request.GetRequestId(), std::ref(request)));
        type_support_->DeserializeRequest(data, request.GetRequest(), request.GetRequestSize());
        return request.GetRequestId();
      }

      void SendResponse(void* data, sequence_number_t request_id)
      {
        Request& request = current_requests_.find(request_id)->second;
        auto resp = type_support_->SerializeResponse(data);
        request.ConsumeResponse(std::move(resp));
        current_requests_.erase(request_id);
      }

      std::string GetName() const
      {
        return name_ + "/" + type_support_->GetServiceSimpleName();
      }

      std::string GetRequestType() const
      {
        return type_support_->GetRequestMessageName();
      }

      std::string GetResponseType() const
      {
        return type_support_->GetResponseMessageName();
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
    };

  } // namespace rmw
} // namespace eCAL
