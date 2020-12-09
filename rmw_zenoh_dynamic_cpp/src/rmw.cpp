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

#include <rmw/rmw.h>

#include <algorithm>
#include <iterator>
#include <chrono>
#include <memory>
#include <mutex>

#include <ecal/ecal.h>

#include <rmw/impl/cpp/macros.hpp>

#include "internal/common.hpp"
#include "internal/node.hpp"
#include "internal/wait_set.hpp"
#include "internal/subscriber.hpp"
#include "internal/publisher.hpp"
#include "internal/service.hpp"
#include "internal/client.hpp"
#include "internal/guard_condition.hpp"
#include "internal/typesupport.hpp"
#include "internal/graph.hpp"

constexpr auto implementation_identifier = "rmw_ecal_dynamic_cpp";

const char* rmw_get_implementation_identifier(void)
{
  return implementation_identifier;
}

const char* rmw_get_serialization_format(void)
{
  return eCAL::rmw::serialization_format;
}

#if ROS_DISTRO >= FOXY
rmw_node_t* rmw_create_node(rmw_context_t* context,
                            const char* name,
                            const char* namespace_,
                            size_t /* domain_id */,
                            bool /* localhost_only */)
#elif ROS_DISTRO == ELOQUENT
rmw_node_t* rmw_create_node(rmw_context_t* context,
                            const char* name,
                            const char* namespace_,
                            size_t /* domain_id */,
                            const rmw_node_security_options_t* security_options,
                            bool /* local_host_only */)
#else
rmw_node_t* rmw_create_node(rmw_context_t* context,
                            const char* name,
                            const char* namespace_,
                            size_t /* domain_id */,
                            const rmw_node_security_options_t* security_options)
#endif
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(namespace_, nullptr);
#if ROS_DISTRO < FOXY
  RMW_CHECK_ARGUMENT_FOR_NULL(security_options, nullptr);
#endif
  CHECK_RMW_IMPLEMENTATION_RET_VALUE(context, nullptr);

  auto rmw_node = rmw_node_allocate();
  auto ecal_node = eCAL::rmw::Graph::CreateNode(namespace_, name);

  rmw_node->context = context;
  rmw_node->implementation_identifier = rmw_get_implementation_identifier();
  rmw_node->name = eCAL::rmw::ConstructCString(name);
  rmw_node->namespace_ = eCAL::rmw::ConstructCString(namespace_);
  rmw_node->data = ecal_node;
  ecal_node->guard_condition = rmw_create_guard_condition(rmw_node->context);

  return rmw_node;
}

rmw_ret_t rmw_destroy_node(rmw_node_t* node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto ecal_node = eCAL::rmw::GetImplementation(node);
  rmw_guard_condition_free(ecal_node->guard_condition);
  eCAL::rmw::Graph::DestroyNode(ecal_node);
  delete[] node->name;
  delete[] node->namespace_;

  rmw_node_free(node);

  return RMW_RET_OK;
}

rmw_ret_t rmw_node_assert_liveliness(const rmw_node_t* /* node */)
{
  UNSUPPORTED;
}

const rmw_guard_condition_t* rmw_node_get_graph_guard_condition(const rmw_node_t* node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  CHECK_RMW_IMPLEMENTATION_RET_VALUE(node, nullptr);

  return eCAL::rmw::GetImplementation(node)->guard_condition;
}

#if ROS_DISTRO >= ELOQUENT
rmw_publisher_t* rmw_create_publisher(const rmw_node_t* node,
                                      const rosidl_message_type_support_t* type_support,
                                      const char* topic_name,
                                      const rmw_qos_profile_t* qos_policies,
                                      const rmw_publisher_options_t* publisher_options)
#else
rmw_publisher_t* rmw_create_publisher(const rmw_node_t* node,
                                      const rosidl_message_type_support_t* type_support,
                                      const char* topic_name,
                                      const rmw_qos_profile_t* qos_policies)
#endif
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
#if ROS_DISTRO >= ELOQUENT
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);
#endif
  CHECK_RMW_IMPLEMENTATION_RET_VALUE(node, nullptr);

  auto ecal_node = eCAL::rmw::GetImplementation(node);
  auto ecal_ts = eCAL::rmw::CreateTypeSupport(type_support);
  auto ecal_qos = eCAL::rmw::CreatePublisherQOS(qos_policies);
  auto ecal_pub = ecal_node->CreatePublisher(topic_name, ecal_ts, ecal_qos);

  auto rmw_pub = rmw_publisher_allocate();
  rmw_pub->implementation_identifier = rmw_get_implementation_identifier();
  rmw_pub->topic_name = eCAL::rmw::ConstructCString(topic_name);
  rmw_pub->data = ecal_pub;
#if ROS_DISTRO >= ELOQUENT
  rmw_pub->options = *publisher_options;
  rmw_pub->can_loan_messages = false;
#endif
  return rmw_pub;
}

rmw_ret_t rmw_destroy_publisher(rmw_node_t* node, rmw_publisher_t* publisher)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto ecal_pub = eCAL::rmw::GetImplementation(publisher);
  eCAL::rmw::GetImplementation(node)->DestroyPublisher(ecal_pub);
  delete[] publisher->topic_name;

  rmw_publisher_free(publisher);

  return RMW_RET_OK;
}

rmw_ret_t rmw_publish(const rmw_publisher_t* publisher,
                      const void* ros_message,
                      rmw_publisher_allocation_t* /* allocation */)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(publisher);

  eCAL::rmw::GetImplementation(publisher)->Publish(ros_message);

  return RMW_RET_OK;
}

rmw_ret_t rmw_publisher_count_matched_subscriptions(const rmw_publisher_t* publisher,
                                                    size_t* subscription_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_count, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(publisher);

  auto actual_topic_name = eCAL::rmw::GetImplementation(publisher)->GetTopicName();
  *subscription_count = eCAL::rmw::Graph::CountSubscribers(actual_topic_name);

  return RMW_RET_OK;
}

rmw_ret_t rmw_publisher_get_actual_qos(const rmw_publisher_t* publisher,
                                       rmw_qos_profile_t* qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(publisher);

  auto ecal_pub = eCAL::rmw::GetImplementation(publisher);
  *qos = ecal_pub->GetRosQOSProfile();

  return RMW_RET_OK;
}

rmw_ret_t rmw_publish_serialized_message(const rmw_publisher_t* publisher,
                                         const rmw_serialized_message_t* serialized_message,
                                         rmw_publisher_allocation_t* /* allocation */)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(publisher);

  auto ecal_pub = eCAL::rmw::GetImplementation(publisher);
  ecal_pub->PublishRaw(serialized_message->buffer, serialized_message->buffer_length);

  return RMW_RET_OK;
}

#if ROS_DISTRO >= FOXY
rmw_ret_t rmw_get_serialized_message_size(const rosidl_message_type_support_t* /* type_support */,
                                          const rosidl_runtime_c__Sequence__bound* /* message_bounds */,
                                          size_t* /* size */)
#else
rmw_ret_t rmw_get_serialized_message_size(const rosidl_message_type_support_t* /* type_support */,
                                          const rosidl_message_bounds_t* /* message_bounds */,
                                          size_t* /* size */)
#endif
{
  UNSUPPORTED;
}

rmw_ret_t rmw_serialize(const void* ros_message,
                        const rosidl_message_type_support_t* type_support,
                        rmw_serialized_message_t* serialized_message)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);

  std::unique_ptr<eCAL::rmw::Serializer> ecal_ser{ eCAL::rmw::CreateSerializer(type_support) };
  auto serialized_bytes = ecal_ser->Serialize(ros_message);
  auto no_of_bytes = serialized_bytes.size();

  serialized_message->buffer_capacity = no_of_bytes;
  serialized_message->buffer_length = no_of_bytes;
  serialized_message->buffer = new uint8_t[no_of_bytes];
  //not very efficient, should find a way to avoid this copy
  std::copy_n(serialized_bytes.data(), no_of_bytes, serialized_message->buffer);

  return RMW_RET_OK;
}

rmw_ret_t rmw_deserialize(const rmw_serialized_message_t* serialized_message,
                          const rosidl_message_type_support_t* type_support,
                          void* ros_message)
{
  std::unique_ptr<eCAL::rmw::Deserializer> ecal_deser{ eCAL::rmw::CreateDeserializer(type_support) };
  ecal_deser->Deserialize(ros_message, serialized_message->buffer, serialized_message->buffer_length);

  return RMW_RET_OK;
}

#if ROS_DISTRO >= ELOQUENT
rmw_subscription_t* rmw_create_subscription(const rmw_node_t* node,
                                            const rosidl_message_type_support_t* type_support,
                                            const char* topic_name,
                                            const rmw_qos_profile_t* qos_policies,
                                            const rmw_subscription_options_t* subscription_options)
#else
rmw_subscription_t* rmw_create_subscription(const rmw_node_t* node,
                                            const rosidl_message_type_support_t* type_support,
                                            const char* topic_name,
                                            const rmw_qos_profile_t* qos_policies,
                                            bool /* ignore_local_publications */)
#endif
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
#if ROS_DISTRO >= ELOQUENT
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_options, nullptr);
#endif
  CHECK_RMW_IMPLEMENTATION_RET_VALUE(node, nullptr);

  auto ecal_node = eCAL::rmw::GetImplementation(node);
  auto ecal_ts = eCAL::rmw::CreateTypeSupport(type_support);
  auto ecal_qos = eCAL::rmw::CreateSubscriberQOS(qos_policies);
  auto ecal_sub = ecal_node->CreateSubscriber(topic_name, ecal_ts, ecal_qos);

  auto rmw_sub = rmw_subscription_allocate();
  rmw_sub->implementation_identifier = rmw_get_implementation_identifier();
  rmw_sub->topic_name = eCAL::rmw::ConstructCString(topic_name);
  rmw_sub->data = ecal_sub;
#if ROS_DISTRO >= ELOQUENT
  rmw_sub->can_loan_messages = false;
#endif

  return rmw_sub;
}

rmw_ret_t rmw_destroy_subscription(rmw_node_t* node, rmw_subscription_t* subscription)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto ecal_sub = eCAL::rmw::GetImplementation(subscription);
  eCAL::rmw::GetImplementation(node)->DestroySubscriber(ecal_sub);
  delete[] subscription->topic_name;

  rmw_subscription_free(subscription);

  return RMW_RET_OK;
}

rmw_ret_t rmw_subscription_count_matched_publishers(const rmw_subscription_t* subscription,
                                                    size_t* publisher_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_count, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(subscription);

  auto actual_topic_name = eCAL::rmw::GetImplementation(subscription)->GetTopicName();
  *publisher_count = eCAL::rmw::Graph::CountPublishers(actual_topic_name);

  return RMW_RET_OK;
}

rmw_ret_t rmw_take(const rmw_subscription_t* subscription,
                   void* ros_message,
                   bool* taken,
                   rmw_subscription_allocation_t* /* allocation */)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(subscription);

  auto ecal_sub = eCAL::rmw::GetImplementation(subscription);
  if (!ecal_sub->HasData())
    return RMW_RET_OK;
  ecal_sub->TakeLatestData(ros_message);
  *taken = true;

  return RMW_RET_OK;
}

rmw_ret_t rmw_take_with_info(const rmw_subscription_t* subscription,
                             void* ros_message,
                             bool* taken,
                             rmw_message_info_t* /* message_info */,
                             rmw_subscription_allocation_t* allocation)
{
  return rmw_take(subscription, ros_message, taken, allocation);
}

#if ROS_DISTRO >= FOXY
rmw_ret_t rmw_take_sequence(const rmw_subscription_t* subscription,
                            size_t count,
                            rmw_message_sequence_t* message_sequence,
                            rmw_message_info_sequence_t* message_info_sequence,
                            size_t* taken,
                            rmw_subscription_allocation_t* /* allocation */)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(subscription);
  if (message_sequence->capacity < count)
  {
    RMW_SET_ERROR_MSG("message_sequence capacity is smaller then count.");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (message_info_sequence->capacity < count)
  {
    RMW_SET_ERROR_MSG("message_info_sequence capacity is smaller then count.");
    return RMW_RET_INVALID_ARGUMENT;
  }

  *taken = 0;
  auto ecal_sub = eCAL::rmw::GetImplementation(subscription);
  while (ecal_sub->HasData() && *taken != count)
  {
    ecal_sub->TakeLatestData(message_sequence->data[*taken]);
    (*taken)++;
  }
  return RMW_RET_OK;
}
#endif

rmw_ret_t rmw_take_serialized_message(const rmw_subscription_t* subscription,
                                      rmw_serialized_message_t* serialized_message,
                                      bool* taken,
                                      rmw_subscription_allocation_t* /* allocation */)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(subscription);

  auto ecal_sub = eCAL::rmw::GetImplementation(subscription);
  if (!ecal_sub->HasData())
    return RMW_RET_OK;

  auto data = ecal_sub->TakeLatestSerializedData();
  serialized_message->buffer = reinterpret_cast<uint8_t*>(data.data);
  serialized_message->buffer_length = data.size;
  serialized_message->buffer_capacity = data.size;
  *taken = true;

  return RMW_RET_OK;
}

rmw_ret_t rmw_take_serialized_message_with_info(const rmw_subscription_t* subscription,
                                                rmw_serialized_message_t* serialized_message,
                                                bool* taken,
                                                rmw_message_info_t* /* message_info */,
                                                rmw_subscription_allocation_t* allocation)
{
  return rmw_take_serialized_message(subscription, serialized_message, taken, allocation);
}

rmw_client_t* rmw_create_client(const rmw_node_t* node,
                                const rosidl_service_type_support_t* type_support,
                                const char* service_name,
                                const rmw_qos_profile_t* qos_policies)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
  CHECK_RMW_IMPLEMENTATION_RET_VALUE(node, nullptr);

  auto ecal_node = eCAL::rmw::GetImplementation(node);
  auto ecal_ts = eCAL::rmw::CreateTypeSupport(type_support);
  auto ecal_qos = eCAL::rmw::CreateClientQOS(qos_policies);
  auto ecal_client = ecal_node->CreateClient(service_name, ecal_ts, ecal_qos);

  auto rmw_client = rmw_client_allocate();
  rmw_client->data = ecal_client;
  rmw_client->implementation_identifier = rmw_get_implementation_identifier();
  rmw_client->service_name = eCAL::rmw::ConstructCString(service_name);

  return rmw_client;
}

rmw_ret_t rmw_destroy_client(rmw_node_t* node, rmw_client_t* client)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto ecal_client = eCAL::rmw::GetImplementation(client);
  eCAL::rmw::GetImplementation(node)->DestroyClient(ecal_client);
  delete[] client->service_name;

  rmw_client_free(client);

  return RMW_RET_OK;
}

rmw_ret_t rmw_send_request(const rmw_client_t* client,
                           const void* ros_request,
                           int64_t* sequence_id)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(sequence_id, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(client);

  auto ecal_client = eCAL::rmw::GetImplementation(client);
  *sequence_id = ecal_client->SendRequest(ros_request);
  if (*sequence_id == -1)
    return RMW_RET_ERROR;

  return RMW_RET_OK;
}

#if ROS_DISTRO >= FOXY
rmw_ret_t
rmw_take_response(const rmw_client_t* client,
                  rmw_service_info_t* request_header,
                  void* ros_response,
                  bool* taken)
#else
rmw_ret_t rmw_take_response(const rmw_client_t* client,
                            rmw_request_id_t* rmw_request_id,
                            void* ros_response,
                            bool* taken)
#endif
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
#if ROS_DISTRO >= FOXY
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
#else
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_request_id, RMW_RET_INVALID_ARGUMENT);
#endif
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(client);

  auto ecal_client = eCAL::rmw::GetImplementation(client);
  if (ecal_client->HasResponse())
  {
#if ROS_DISTRO >= FOXY
    request_header->request_id.sequence_number = ecal_client->TakeResponse(ros_response);
#else
    rmw_request_id->sequence_number = ecal_client->TakeResponse(ros_response);
#endif
    * taken = true;
  }

  return RMW_RET_OK;
}

rmw_service_t* rmw_create_service(const rmw_node_t* node,
                                  const rosidl_service_type_support_t* type_support,
                                  const char* service_name,
                                  const rmw_qos_profile_t* qos_policies)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
  CHECK_RMW_IMPLEMENTATION_RET_VALUE(node, nullptr);

  auto ecal_node = eCAL::rmw::GetImplementation(node);
  auto ecal_ts = eCAL::rmw::CreateTypeSupport(type_support);
  auto ecal_qos = eCAL::rmw::CreateServiceQOS(qos_policies);
  auto ecal_service = ecal_node->CreateService(service_name, ecal_ts, ecal_qos);

  auto rmw_service = rmw_service_allocate();
  rmw_service->data = ecal_service;
  rmw_service->implementation_identifier = rmw_get_implementation_identifier();
  rmw_service->service_name = eCAL::rmw::ConstructCString(service_name);

  return rmw_service;
}

rmw_ret_t rmw_destroy_service(rmw_node_t* node, rmw_service_t* service)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto ecal_service = eCAL::rmw::GetImplementation(service);
  eCAL::rmw::GetImplementation(node)->DestroyService(ecal_service);
  delete[] service->service_name;

  rmw_service_free(service);

  return RMW_RET_OK;
}

#if ROS_DISTRO >= FOXY
rmw_ret_t rmw_take_request(const rmw_service_t* service,
                           rmw_service_info_t* request_header,
                           void* ros_request,
                           bool* taken)
#else
rmw_ret_t rmw_take_request(const rmw_service_t* service,
                           rmw_request_id_t* request_header,
                           void* ros_request,
                           bool* taken)
#endif
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(service);

  auto ecal_service = eCAL::rmw::GetImplementation(service);
  if (ecal_service->HasRequest())
  {
#if ROS_DISTRO >= FOXY
    request_header->request_id.sequence_number = ecal_service->TakeRequest(ros_request);
#else
    request_header->sequence_number = ecal_service->TakeRequest(ros_request);
#endif
    * taken = true;
  }

  return RMW_RET_OK;
}

rmw_ret_t rmw_send_response(const rmw_service_t* service,
                            rmw_request_id_t* request_header,
                            void* ros_response)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(service);

  auto ecal_service = eCAL::rmw::GetImplementation(service);
  ecal_service->SendResponse(ros_response, request_header->sequence_number);

  return RMW_RET_OK;
}

rmw_guard_condition_t* rmw_create_guard_condition(rmw_context_t* context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  CHECK_RMW_IMPLEMENTATION_RET_VALUE(context, nullptr);

  auto rmw_gc = rmw_guard_condition_allocate();
  rmw_gc->context = context;
  rmw_gc->data = new eCAL::rmw::GuardCondition;
  rmw_gc->implementation_identifier = rmw_get_implementation_identifier();

  return rmw_gc;
}

rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t* guard_condition)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(guard_condition);

  delete eCAL::rmw::GetImplementation(guard_condition);
  rmw_guard_condition_free(guard_condition);

  return RMW_RET_OK;
}

rmw_ret_t rmw_trigger_guard_condition(const rmw_guard_condition_t* guard_condition)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(guard_condition);

  eCAL::rmw::GetImplementation(guard_condition)->Trigger();

  return RMW_RET_OK;
}

rmw_wait_set_t* rmw_create_wait_set(rmw_context_t* context, size_t /* max_conditions */)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  CHECK_RMW_IMPLEMENTATION_RET_VALUE(context, nullptr);

  auto rmw_ws = rmw_wait_set_allocate();
  rmw_ws->data = new eCAL::rmw::WaitSet;
  rmw_ws->implementation_identifier = rmw_get_implementation_identifier();

  return rmw_ws;
}

rmw_ret_t rmw_destroy_wait_set(rmw_wait_set_t* wait_set)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(wait_set);

  delete eCAL::rmw::GetImplementation(wait_set);
  rmw_wait_set_free(wait_set);

  return RMW_RET_OK;
}

bool has_data(rmw_subscriptions_t* subscriptions,
              rmw_guard_conditions_t* guard_conditions,
              rmw_services_t* services,
              rmw_clients_t* clients,
              rmw_events_t* events)
{
  for (size_t i = 0; i < subscriptions->subscriber_count; ++i)
  {
    const auto ecal_sub = eCAL::rmw::GetImplementation(subscriptions, i);
    if (ecal_sub->HasData())
    {
      return true;
    }
  }

  for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i)
  {
    const auto ecal_gc = eCAL::rmw::GetImplementation(guard_conditions, i);
    if (ecal_gc->Triggered())
    {
      return true;
    }
  }

  for (size_t i = 0; i < services->service_count; ++i)
  {
    const auto ecal_service = eCAL::rmw::GetImplementation(services, i);
    if (ecal_service->HasRequest())
    {
      return true;
    }
  }

  for (size_t i = 0; i < clients->client_count; ++i)
  {
    const auto ecal_client = eCAL::rmw::GetImplementation(clients, i);
    if (ecal_client->HasResponse())
    {
      return true;
    }
  }

  for (size_t i = 0; i < events->event_count; ++i)
  {
    const auto ecal_event = eCAL::rmw::GetImplementation(events, i);
    if (ecal_event->Triggered())
    {
      return true;
    }
  }

  return false;
}

void attach_wait_set(rmw_subscriptions_t* subscriptions,
                     rmw_guard_conditions_t* guard_conditions,
                     rmw_services_t* services,
                     rmw_clients_t* clients,
                     rmw_events_t* events,
                     eCAL::rmw::WaitSet* ecal_ws)
{
  for (size_t i = 0; i < subscriptions->subscriber_count; ++i)
  {
    auto ecal_sub = eCAL::rmw::GetImplementation(subscriptions, i);
    ecal_sub->AttachWaitSet(ecal_ws);
  }

  for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i)
  {
    auto ecal_gc = eCAL::rmw::GetImplementation(guard_conditions, i);
    ecal_gc->AttachWaitSet(ecal_ws);
  }

  for (size_t i = 0; i < services->service_count; ++i)
  {
    auto ecal_service = eCAL::rmw::GetImplementation(services, i);
    ecal_service->AttachWaitSet(ecal_ws);
  }

  for (size_t i = 0; i < clients->client_count; ++i)
  {
    auto ecal_client = eCAL::rmw::GetImplementation(clients, i);
    ecal_client->AttachWaitSet(ecal_ws);
  }

  for (size_t i = 0; i < events->event_count; ++i)
  {
    auto ecal_event = eCAL::rmw::GetImplementation(events, i);
    ecal_event->AttachWaitSet(ecal_ws);
  }
}

void detach_wait_set(rmw_subscriptions_t* subscriptions,
                     rmw_guard_conditions_t* guard_conditions,
                     rmw_services_t* services,
                     rmw_clients_t* clients,
                     rmw_events_t* events)
{
  for (size_t i = 0; i < subscriptions->subscriber_count; ++i)
  {
    auto ecal_sub = eCAL::rmw::GetImplementation(subscriptions, i);
    ecal_sub->DetachWaitSet();
    if (!ecal_sub->HasData())
    {
      subscriptions->subscribers[i] = nullptr;
    }
  }

  for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i)
  {
    auto ecal_gc = eCAL::rmw::GetImplementation(guard_conditions, i);
    ecal_gc->DetachWaitSet();
    if (!ecal_gc->TakeTriggered())
    {
      guard_conditions->guard_conditions[i] = nullptr;
    }
  }

  for (size_t i = 0; i < services->service_count; ++i)
  {
    auto ecal_ser = eCAL::rmw::GetImplementation(services, i);
    ecal_ser->DetachWaitSet();
    if (!ecal_ser->HasRequest())
    {
      services->services[i] = nullptr;
    }
  }

  for (size_t i = 0; i < clients->client_count; ++i)
  {
    auto ecal_client = eCAL::rmw::GetImplementation(clients, i);
    ecal_client->DetachWaitSet();
    if (!ecal_client->HasResponse())
    {
      clients->clients[i] = nullptr;
    }
  }

  for (size_t i = 0; i < events->event_count; ++i)
  {
    auto ecal_event = eCAL::rmw::GetImplementation(events, i);
    ecal_event->DetachWaitSet();
    if (!ecal_event->Triggered())
    {
      events->events[i] = nullptr;
    }
  }
}

rmw_ret_t rmw_wait(rmw_subscriptions_t* subscriptions,
                   rmw_guard_conditions_t* guard_conditions,
                   rmw_services_t* services,
                   rmw_clients_t* clients,
                   rmw_events_t* events,
                   rmw_wait_set_t* wait_set,
                   const rmw_time_t* wait_timeout)
{
  auto ecal_ws = eCAL::rmw::GetImplementation(wait_set);
  bool timed_out = false;

  attach_wait_set(subscriptions, guard_conditions, services, clients, events, ecal_ws);

  std::unique_lock<std::mutex> lock(ecal_ws->condition_mutex);
  while (!has_data(subscriptions, guard_conditions, services, clients, events))
  {
    if (wait_timeout)
    {
      auto n = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(wait_timeout->sec)) + std::chrono::nanoseconds(wait_timeout->nsec);
      timed_out = !ecal_ws->Wait(lock, n);
      if (timed_out)
        break;
    }
    else
    {
      ecal_ws->Wait(lock);
    }
  }
  lock.unlock();

  detach_wait_set(subscriptions, guard_conditions, services, clients, events);

  return timed_out ? RMW_RET_TIMEOUT : RMW_RET_OK;
}

rmw_ret_t rmw_get_node_names(const rmw_node_t* node,
                             rcutils_string_array_t* node_names,
                             rcutils_string_array_t* node_namespaces)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_names, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespaces, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto nodes = eCAL::rmw::Graph::GetNodes();

  auto allocator = rcutils_get_default_allocator();
  auto init_success = rcutils_string_array_init(node_names, nodes.size(), &allocator);
  if (init_success != RMW_RET_OK)
  {
    RMW_SET_ERROR_MSG("Failed to initialize node_names.");
    return RMW_RET_ERROR;
  }
  init_success = rcutils_string_array_init(node_namespaces, nodes.size(), &allocator);
  if (init_success != RMW_RET_OK)
  {
    RMW_SET_ERROR_MSG("Failed to initialize node_namespaces.");
    return RMW_RET_ERROR;
  }

  std::transform(nodes.begin(), nodes.end(), eCAL::rmw::RosArray::Begin(*node_names),
    [](auto& node) {
      return eCAL::rmw::ConstructCString(node.name);
    });
  std::transform(nodes.begin(), nodes.end(), eCAL::rmw::RosArray::Begin(*node_namespaces),
    [](auto& node) {
      return eCAL::rmw::ConstructCString(node.name_space);
    });

  return RMW_RET_OK;
}

#if ROS_DISTRO >= FOXY
rmw_ret_t rmw_get_node_names_with_enclaves(const rmw_node_t* /* node */,
                                           rcutils_string_array_t* /* node_names */,
                                           rcutils_string_array_t* /* node_namespaces */,
                                           rcutils_string_array_t* /* enclaves */)
{
  UNSUPPORTED;
}
#endif

rmw_ret_t rmw_count_publishers(const rmw_node_t* node,
                               const char* topic_name,
                               size_t* count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  *count = eCAL::rmw::Graph::CountPublishers(topic_name);

  return RMW_RET_OK;
}

rmw_ret_t rmw_count_subscribers(const rmw_node_t* node,
                                const char* topic_name,
                                size_t* count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  *count = eCAL::rmw::Graph::CountSubscribers(topic_name);

  return RMW_RET_OK;
}

rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t* publisher, rmw_gid_t* gid)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(publisher);

  gid->implementation_identifier = rmw_get_implementation_identifier();
  std::fill(std::begin(gid->data), std::end(gid->data), uint8_t{ 0 });

  return RMW_RET_OK;
}

rmw_ret_t rmw_compare_gids_equal(const rmw_gid_t* gid1, const rmw_gid_t* gid2, bool* result)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(gid1, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid2, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(result, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(gid1);

  *result = std::equal(std::begin(gid1->data), std::end(gid1->data), std::begin(gid2->data));
  return RMW_RET_OK;
}

rmw_ret_t rmw_service_server_is_available(const rmw_node_t* node,
                                          const rmw_client_t* client,
                                          bool* is_available)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(is_available, RMW_RET_INVALID_ARGUMENT);
  CHECK_RMW_IMPLEMENTATION(node);

  auto ecal_client = eCAL::rmw::GetImplementation(client);
  *is_available = ecal_client->IsServiceAvailable();

  return RMW_RET_OK;
}

rmw_ret_t rmw_set_log_severity(rmw_log_severity_t severity)
{
  switch (severity)
  {
  case RMW_LOG_SEVERITY_DEBUG:
    eCAL::Logging::SetLogLevel(eCAL_Logging_eLogLevel::log_level_debug1);
    break;
  case RMW_LOG_SEVERITY_INFO:
    eCAL::Logging::SetLogLevel(eCAL_Logging_eLogLevel::log_level_info);
    break;
  case RMW_LOG_SEVERITY_WARN:
    eCAL::Logging::SetLogLevel(eCAL_Logging_eLogLevel::log_level_warning);
    break;
  case RMW_LOG_SEVERITY_ERROR:
    eCAL::Logging::SetLogLevel(eCAL_Logging_eLogLevel::log_level_error);
    break;
  case RMW_LOG_SEVERITY_FATAL:
    eCAL::Logging::SetLogLevel(eCAL_Logging_eLogLevel::log_level_fatal);
    break;
  }
  return RMW_RET_OK;
}

rmw_ret_t rmw_subscription_get_actual_qos(const rmw_subscription_t* subscription,
                                          rmw_qos_profile_t* qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_ERROR);
  CHECK_RMW_IMPLEMENTATION(subscription);

  auto ecal_sub = eCAL::rmw::GetImplementation(subscription);
  *qos = ecal_sub->GetRosQOSProfile();

  return RMW_RET_OK;
}

#if ROS_DISTRO >= FOXY
rmw_ret_t rmw_init_publisher_allocation(const rosidl_message_type_support_t* /* type_support */,
                                        const rosidl_runtime_c__Sequence__bound* /* message_bounds */,
                                        rmw_publisher_allocation_t* /* allocation */)
#else
rmw_ret_t rmw_init_publisher_allocation(const rosidl_message_type_support_t* /* type_support */,
                                        const rosidl_message_bounds_t* /* message_bounds */,
                                        rmw_publisher_allocation_t* /* allocation */)
#endif
{
  UNSUPPORTED;
}

rmw_ret_t rmw_fini_publisher_allocation(rmw_publisher_allocation_t* /* allocation */)
{
  UNSUPPORTED;
}

rmw_ret_t rmw_publisher_assert_liveliness(const rmw_publisher_t* /* publisher */)
{
  UNSUPPORTED;
}

#if ROS_DISTRO >= FOXY
rmw_ret_t rmw_init_subscription_allocation(const rosidl_message_type_support_t* /* type_support */,
                                           const rosidl_runtime_c__Sequence__bound* /* message_bounds */,
                                           rmw_subscription_allocation_t* /* allocation */)
#else
rmw_ret_t rmw_init_subscription_allocation(const rosidl_message_type_support_t* /* type_support */,
                                           const rosidl_message_bounds_t* /* message_bounds */,
                                           rmw_subscription_allocation_t* /* allocation */)
#endif
{
  UNSUPPORTED;
}

rmw_ret_t rmw_fini_subscription_allocation(rmw_subscription_allocation_t* /* allocation */)
{
  UNSUPPORTED;
}

rmw_ret_t rmw_take_loaned_message(const rmw_subscription_t* /* subscription */,
                                  void** /* loaned_message */,
                                  bool* /* taken */,
                                  rmw_subscription_allocation_t* /* allocation */)
{
  UNSUPPORTED;
}

rmw_ret_t rmw_take_loaned_message_with_info(const rmw_subscription_t* /* subscription */,
                                            void** /* loaned_message */,
                                            bool* /* taken */,
                                            rmw_message_info_t* /* message_info */,
                                            rmw_subscription_allocation_t* /* allocation */)
{
  UNSUPPORTED;
}

rmw_ret_t rmw_return_loaned_message_from_subscription(const rmw_subscription_t* /* subscription */,
                                                      void* /* loaned_message */)
{
  UNSUPPORTED;
}

rmw_ret_t rmw_borrow_loaned_message(const rmw_publisher_t* /* publisher */,
                                    const rosidl_message_type_support_t* /* type_support */,
                                    void** /* ros_message */)
{
  UNSUPPORTED;
}

rmw_ret_t rmw_publish_loaned_message(const rmw_publisher_t* /* publisher */,
                                     void* /* ros_message */,
                                     rmw_publisher_allocation_t* /* allocation */)
{
  UNSUPPORTED;
}

rmw_ret_t rmw_return_loaned_message_from_publisher(const rmw_publisher_t* /* publisher */,
                                                   void* /* loaned_message */)
{
  UNSUPPORTED;
}
