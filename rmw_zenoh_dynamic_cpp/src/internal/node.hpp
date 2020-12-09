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
#include <functional>
#include <unordered_set>

#include <ecal/ecal.h>

#include <rmw/types.h>

#include "internal/common.hpp"
#include "internal/typesupport.hpp"
#include "internal/qos.hpp"
#include "internal/publisher.hpp"
#include "internal/subscriber.hpp"
#include "internal/service.hpp"
#include "internal/client.hpp"

#include "publisher.pb.h"
#include "subscriber.pb.h"
#include "service.pb.h"
#include "client.pb.h"

namespace eCAL
{
  namespace rmw
  {
    class Node
    {
      eCAL::CServiceServer            query_service_;

      std::unordered_set<Subscriber*> subscribers_;
      std::unordered_set<Publisher*>  publishers_;
      std::unordered_set<Service*>    services_;
      std::unordered_set<Client*>     clients_;

      int OnGetSubscribers(const std::string& /* method */,
                           const std::string& /* req_type */, const std::string& /* resp_type */,
                           const std::string& /* request */, std::string& response)
      {
        pb::GraphInfo::Subscribers msg;

        for (auto sub : subscribers_)
        {
          auto data{ msg.add_subscribers() };

          data->set_name(sub->GetTopicName());
          data->set_type(sub->GetTopicType());
        }

        msg.SerializeToString(&response);

        return 1;
      }

      int OnGetPublishers(const std::string& /* method */,
                          const std::string& /* req_type */, const std::string& /* resp_type */,
                          const std::string& /* request */, std::string& response)
      {
        pb::GraphInfo::Publishers msg;

        for (auto pub : publishers_)
        {
          auto data{ msg.add_publishers() };
          pub->GetRosQOSProfile();
          data->set_name(pub->GetTopicName());
          data->set_type(pub->GetTopicType());
        }

        msg.SerializeToString(&response);

        return 1;
      }

      int OnGetServices(const std::string& /* method */,
                        const std::string& /* req_type */, const std::string& /* resp_type */,
                        const std::string& /* request */, std::string& response)
      {
        pb::GraphInfo::Services msg;

        for (auto ser : services_)
        {
          auto data{ msg.add_services() };
          data->set_name(ser->GetName());
          data->set_request_type(ser->GetRequestType());
          data->set_response_type(ser->GetResponseType());
        }

        msg.SerializeToString(&response);

        return 1;
      }

      int OnGetClients(const std::string& /* method */,
                       const std::string& /* req_type */, const std::string& /* resp_type */,
                       const std::string& /* request */, std::string& response)
      {
        pb::GraphInfo::Clients msg;

        for (auto cli : clients_)
        {
          auto data{ msg.add_clients() };
          data->set_name(cli->GetName());
          data->set_request_type(cli->GetRequestType());
          data->set_response_type(cli->GetResponseType());
        }

        msg.SerializeToString(&response);

        return 1;
      }

    public:
      rmw_guard_condition_t* guard_condition;

      Node(const std::string& namespace_, const std::string& name) : query_service_{ BuildQueryServiceName(namespace_, name) }
      {
        using namespace std::placeholders;

        query_service_.AddMethodCallback("GetSubscribers", "Empty", "Subscribers.pb",
          std::bind(&Node::OnGetSubscribers, this, _1, _2, _3, _4, _5));
        query_service_.AddMethodCallback("GetPublishers", "Empty", "Publishers.pb",
          std::bind(&Node::OnGetPublishers, this, _1, _2, _3, _4, _5));
        query_service_.AddMethodCallback("GetServices", "Empty", "Services.pb",
          std::bind(&Node::OnGetServices, this, _1, _2, _3, _4, _5));
        query_service_.AddMethodCallback("GetClients", "Empty", "Clients.pb",
          std::bind(&Node::OnGetClients, this, _1, _2, _3, _4, _5));
      }

      Subscriber* CreateSubscriber(const std::string& topic_name, MessageTypeSupport* ts, const SubscriberQOS& qos)
      {
        auto sub = new Subscriber{ topic_name, ts, qos };
        subscribers_.insert(sub);
        return sub;
      }

      void DestroySubscriber(Subscriber* sub)
      {
        if (subscribers_.find(sub) != subscribers_.end())
        {
          subscribers_.erase(sub);
          delete sub;
        }
      }

      Publisher* CreatePublisher(const std::string& topic_name, MessageTypeSupport* ts, const PublisherQOS& qos)
      {
        auto pub = new Publisher{ topic_name, ts, qos };
        publishers_.insert(pub);
        return pub;
      }

      void DestroyPublisher(Publisher* pub)
      {
        if (publishers_.find(pub) != publishers_.end())
        {
          publishers_.erase(pub);
          delete pub;
        }
      }

      Service* CreateService(const std::string& name, ServiceTypeSupport* ts, const ServiceQOS& qos)
      {
        auto ser = new Service{ name, ts, qos };
        services_.insert(ser);
        return ser;
      }

      void DestroyService(Service* ser)
      {
        if (services_.find(ser) != services_.end())
        {
          services_.erase(ser);
          delete ser;
        }
      }

      Client* CreateClient(const std::string& name, ServiceTypeSupport* ts, const ClientQOS& qos)
      {
        auto cli = new Client{ name, ts, qos };
        clients_.insert(cli);
        return cli;
      }

      void DestroyClient(Client* cli)
      {
        if (clients_.find(cli) != clients_.end())
        {
          clients_.erase(cli);
          delete cli;
        }
      }
    };

  } // namespace rmw
} // namespace eCAL
