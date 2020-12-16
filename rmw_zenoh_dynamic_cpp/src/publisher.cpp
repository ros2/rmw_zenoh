// Copyright 2020 ADLINK, Inc.
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

#include <string>

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/validate_full_topic_name.h"

#include "rcpputils/scope_exit.hpp"

#include "rmw_zenoh_common_cpp/custom_session_info.hpp"
#include "rmw_zenoh_common_cpp/custom_publisher_info.hpp"
#include "rmw_zenoh_common_cpp/names.hpp"
#include "rmw_zenoh_common_cpp/namespace_prefix.hpp"
// #include "rmw_zenoh_common_cpp/qos.hpp"
#include "rmw_zenoh_common_cpp/rmw_common.hpp"

#include "rmw_zenoh_dynamic_cpp/identifier.hpp"
#include "rmw_zenoh_dynamic_cpp/publisher.hpp"

#include "type_support_common.hpp"
#include "type_support_registry.hpp"

using BaseTypeSupport = rmw_zenoh_dynamic_cpp::BaseTypeSupport;
using TypeSupportProxy = rmw_zenoh_dynamic_cpp::TypeSupportProxy;

rmw_publisher_t *
rmw_zenoh_dynamic_cpp::create_publisher(
  const CustomSessionInfo * session_info,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies,
  const rmw_publisher_options_t * publisher_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(session_info, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  if (0 == strlen(topic_name)) {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
  if (!qos_policies->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic name: %s", reason);
      return nullptr;
    }
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);

  zn_session_t * session = session_info->session;
  RMW_CHECK_ARGUMENT_FOR_NULL(session, nullptr);

  const rosidl_message_type_support_t * type_support = get_message_typesupport_handle(
    type_supports, rosidl_typesupport_introspection_c__identifier);
  if (!type_support) {
    type_support = get_message_typesupport_handle(
      type_supports, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    if (!type_support) {
      RMW_SET_ERROR_MSG("type support not from this implementation");
      return nullptr;
    }
  }

  if (!is_valid_qos(*qos_policies)) {
    return nullptr;
  }

  CustomPublisherInfo * info = nullptr;
  rmw_publisher_t * rmw_publisher = nullptr;

  info = new (std::nothrow) CustomPublisherInfo();
  if (!info) {
    RMW_SET_ERROR_MSG("failed to allocate CustomPublisherInfo");
    return nullptr;
  }
  auto cleanup_info = rcpputils::make_scope_exit(
    [info, session]() {
      if (info->type_support_) {
        _unregister_type(session, info->type_support_);
      }
      delete info;
    });

  TypeSupportRegistry & type_registry = TypeSupportRegistry::get_instance();
  auto type_impl = type_registry.get_message_type_support(type_support);
  if (!type_impl) {
    RMW_SET_ERROR_MSG("failed to allocate type support");
    return nullptr;
  }
  auto return_type_support = rcpputils::make_scope_exit(
    [&type_registry, type_support]() {
      type_registry.return_message_type_support(type_support);
    });

  info->typesupport_identifier_ = type_support->typesupport_identifier;
  info->type_support_impl_ = type_impl;

  std::string type_name = _create_type_name(
    type_support->data, info->typesupport_identifier_);
  if (
    !Domain::getRegisteredType(
      participant, type_name.c_str(),
      reinterpret_cast<TopicDataType **>(&info->type_support_)))
  {
    info->type_support_ = new (std::nothrow) TypeSupportProxy(type_impl);
    if (!info->type_support_) {
      RMW_SET_ERROR_MSG("failed to allocate TypeSupportProxy");
      return nullptr;
    }
    _register_type(participant, info->type_support_);
  }

  if (!session_info->leave_middleware_default_qos) {
    publisherParam.qos.m_publishMode.kind = eprosima::zenoh::ASYNCHRONOUS_PUBLISH_MODE;
    publisherParam.historyMemoryPolicy =
      eprosima::zenoh::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
  }

  publisherParam.topic.topicKind = eprosima::zenoh::rtps::NO_KEY;
  publisherParam.topic.topicDataType = type_name;
  publisherParam.topic.topicName = _create_topic_name(qos_policies, ros_topic_prefix, topic_name);

  // 1 Heartbeat every 10ms
  // publisherParam.times.heartbeatPeriod.seconds = 0;
  // publisherParam.times.heartbeatPeriod.fraction = 42949673;

  // 300000 bytes each 10ms
  // ThroughputControllerDescriptor throughputController{3000000, 10};
  // publisherParam.throughputController = throughputController;

  if (!get_datawriter_qos(*qos_policies, publisherParam)) {
    return nullptr;
  }

  info->listener_ = new (std::nothrow) PubListener(info);
  if (!info->listener_) {
    RMW_SET_ERROR_MSG("create_publisher() could not create publisher listener");
    return nullptr;
  }

  info->publisher_ = Domain::createPublisher(participant, publisherParam, info->listener_);
  if (!info->publisher_) {
    RMW_SET_ERROR_MSG("create_publisher() could not create publisher");
    return nullptr;
  }

  info->publisher_gid = rmw_zenoh_common_cpp::create_rmw_gid(
    eclipse_zenoh_identifier, info->publisher_->getGuid());

  rmw_publisher = rmw_publisher_allocate();
  if (!rmw_publisher) {
    RMW_SET_ERROR_MSG("failed to allocate publisher");
    return nullptr;
  }
  auto cleanup_publisher = rcpputils::make_scope_exit(
    [rmw_publisher]() {
      rmw_free(const_cast<char *>(rmw_publisher->topic_name));
      rmw_publisher_free(rmw_publisher);
    });

  rmw_publisher->can_loan_messages = false;
  rmw_publisher->implementation_identifier = eclipse_zenoh_identifier;
  rmw_publisher->data = info;

  rmw_publisher->topic_name = static_cast<char *>(rmw_allocate(strlen(topic_name) + 1));
  if (!rmw_publisher->topic_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for publisher topic name");
    return nullptr;
  }
  memcpy(const_cast<char *>(rmw_publisher->topic_name), topic_name, strlen(topic_name) + 1);

  rmw_publisher->options = *publisher_options;

  cleanup_publisher.cancel();
  cleanup_info.cancel();
  return_type_support.cancel();
  return rmw_publisher;
}
