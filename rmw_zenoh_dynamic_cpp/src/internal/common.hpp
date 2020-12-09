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
#include <cstring>
#include <atomic>

#include <rmw/types.h>
#include <rmw/event.h>
#include <rmw/ret_types.h>
#include <rmw/error_handling.h>
#include <rmw/impl/cpp/macros.hpp>

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_c/message_introspection.h>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>
#include <rosidl_typesupport_introspection_c/service_introspection.h>

#include <ecal/ecal.h>
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4127 4146 4800)
#endif
#include <ecal/pb/monitoring.pb.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include "internal/ros_array_iterator.hpp"

namespace eCAL
{
namespace rmw
{

class Node;
class GuardCondition;
class WaitSet;
class Publisher;
class Service;
class Subscriber;
class Client;
class Event;

//Represents client/service sequence number.
using sequence_number_t = int64_t;
//Represents data type for serialized array size.
using array_size_t = uint64_t;
//Represents arbitrary ros message type, used for consistency
//of serialization/deserialization template functions.
struct ros_message_t;

inline Node *GetImplementation(const rmw_node_t *node)
{
	return static_cast<Node *>(node->data);
}

inline GuardCondition *GetImplementation(const rmw_guard_condition_t *guard_condition)
{
	return static_cast<GuardCondition *>(guard_condition->data);
}

inline WaitSet *GetImplementation(const rmw_wait_set_t *wait_set)
{
	return static_cast<WaitSet *>(wait_set->data);
}

inline Publisher *GetImplementation(const rmw_publisher_t *publisher)
{
	return static_cast<Publisher *>(publisher->data);
}

inline Subscriber *GetImplementation(const rmw_subscription_t *subscription)
{
	return static_cast<Subscriber *>(subscription->data);
}

inline Service *GetImplementation(const rmw_service_t *service)
{
	return static_cast<Service *>(service->data);
}

inline Client *GetImplementation(const rmw_client_t *client)
{
	return static_cast<Client *>(client->data);
}

inline Event *GetImplementation(const rmw_event_t *event)
{
	return static_cast<Event *>(event->data);
}

inline GuardCondition *GetImplementation(const rmw_guard_conditions_t *guard_conditions, size_t index)
{
	return static_cast<GuardCondition *>(guard_conditions->guard_conditions[index]);
}

inline Subscriber *GetImplementation(const rmw_subscriptions_t *subscriptions, size_t index)
{
	return static_cast<Subscriber *>(subscriptions->subscribers[index]);
}

inline Service *GetImplementation(const rmw_services_t *services, size_t index)
{
	return static_cast<Service *>(services->services[index]);
}

inline Client *GetImplementation(const rmw_clients_t *clients, size_t index)
{
	return static_cast<Client *>(clients->clients[index]);
}

inline Event *GetImplementation(const rmw_events_t *events, size_t index)
{
	return static_cast<Event *>(events->events[index]);
}

inline const rosidl_typesupport_introspection_cpp::MessageMembers *GetMembers(
	const rosidl_typesupport_introspection_cpp::MessageMember *member)
{
	return static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member->members_->data);
}

inline const rosidl_typesupport_introspection_c__MessageMembers *GetMembers(
	const rosidl_typesupport_introspection_c__MessageMember *member)
{
	return static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(member->members_->data);
}

inline const rosidl_typesupport_introspection_cpp::MessageMembers *GetCppMembers(
	const rosidl_message_type_support_t *type_support)
{
	return static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(type_support->data);
}

inline const rosidl_typesupport_introspection_c__MessageMembers *GetCMembers(
	const rosidl_message_type_support_t *type_support)
{
	return static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(type_support->data);
}

inline const rosidl_typesupport_introspection_cpp::ServiceMembers *GetCppMembers(
	const rosidl_service_type_support_t *type_support)
{
	return static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(type_support->data);
}

inline const rosidl_typesupport_introspection_c__ServiceMembers *GetCMembers(
	const rosidl_service_type_support_t *type_support)
{
	return static_cast<const rosidl_typesupport_introspection_c__ServiceMembers *>(type_support->data);
}

inline sequence_number_t GenerateSequenceNumber()
{
	static std::atomic<sequence_number_t> sequence_number{0};
	return sequence_number++;
}

inline size_t GetSequenceDataSize(const std::string &sequence)
{
	return sequence.size() - sizeof(sequence_number_t);
}

inline sequence_number_t GetSequenceNumber(const std::string &sequence)
{
	return *reinterpret_cast<const sequence_number_t *>(sequence.data() + GetSequenceDataSize(sequence));
}

inline char *ConstructCString(const char *value)
{
	size_t src_len{std::strlen(value) + 1};
	char *dest = new char[src_len];
	std::strcpy(dest, value);
	return dest;
}

inline std::string ReplaceString(std::string subject, const std::string& search,
                          const std::string& replace) 
{
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos) 
	{
         subject.replace(pos, search.length(), replace);
         pos += replace.length();
    }
    return subject;
}

inline char *ConstructCString(const std::string &value)
{
	return ConstructCString(value.c_str());
}

#define UNIMPLEMENTED                                                                   \
	RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("%s has not been implemented yet.", __func__); \
	return RMW_RET_UNSUPPORTED

#define UNSUPPORTED                                                         \
	RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("%s is not supported.", __func__); \
	return RMW_RET_UNSUPPORTED

#define CHECK_RMW_IMPLEMENTATION(ElementName)                                \
	CHECK_RMW_IMPLEMENTATION_RET_VALUE(ElementName,                          \
									   RMW_RET_INCORRECT_RMW_IMPLEMENTATION)

#define CHECK_RMW_IMPLEMENTATION_RET_VALUE(ElementName, ReturnValue) \
	RMW_CHECK_TYPE_IDENTIFIERS_MATCH(                                \
		ElementName,                                                 \
		ElementName->implementation_identifier,                      \
		rmw_get_implementation_identifier(),                         \
		return ReturnValue)

} // namespace rmw
} // namespace eCAL
