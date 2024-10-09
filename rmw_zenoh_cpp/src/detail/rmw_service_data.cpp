// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "rmw_service_data.hpp"

#include <fastcdr/FastBuffer.h>

#include <cinttypes>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "attachment_helpers.hpp"
#include "cdr.hpp"
#include "rmw_context_impl_s.hpp"
#include "message_type_support.hpp"
#include "logging_macros.hpp"
#include "qos.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/impl/cpp/macros.hpp"

namespace rmw_zenoh_cpp
{
///==============================================================================
void service_data_handler(const z_query_t * query, void * data)
{
  z_owned_str_t keystr = z_keyexpr_to_string(z_query_keyexpr(query));
  auto drop_keystr = rcpputils::make_scope_exit(
    [&keystr]() {
      z_drop(z_move(keystr));
    });

  ServiceData * service_data =
    static_cast<ServiceData *>(data);
  if (service_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain ServiceData from data for "
      "service for %s",
      z_loan(keystr)
    );
    return;
  }

  service_data->add_new_query(std::make_unique<ZenohQuery>(query));
}

///=============================================================================
std::shared_ptr<ServiceData> ServiceData::make(
  z_session_t session,
  const rmw_node_t * const node,
  liveliness::NodeInfo node_info,
  std::size_t node_id,
  std::size_t service_id,
  const std::string & service_name,
  const rosidl_service_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile)
{
  // Adapt any 'best available' QoS options
  rmw_qos_profile_t adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = QoS::get().best_available_qos(
    nullptr, nullptr, &adapted_qos_profile, nullptr);
  if (RMW_RET_OK != ret) {
    RMW_SET_ERROR_MSG("Failed to obtain adapted_qos_profile.");
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  const rosidl_type_hash_t * type_hash = type_support->get_type_hash_func(type_support);
  auto service_members = static_cast<const service_type_support_callbacks_t *>(type_support->data);
  auto request_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->request_members_->data);
  auto response_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->response_members_->data);
  auto request_type_support = std::make_unique<RequestTypeSupport>(service_members);
  auto response_type_support = std::make_unique<ResponseTypeSupport>(service_members);

  // Note: Service request/response types will contain a suffix Request_ or Response_.
  // We remove the suffix when appending the type to the liveliness tokens for
  // better reusability within GraphCache.
  std::string service_type = response_type_support->get_name();
  size_t suffix_substring_position = service_type.find("Response_");
  if (std::string::npos != suffix_substring_position) {
    service_type = service_type.substr(0, suffix_substring_position);
  } else {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unexpected type %s for service %s. Report this bug",
      service_type.c_str(), service_name.c_str());
    return nullptr;
  }

  // Convert the type hash to a string so that it can be included in the keyexpr.
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(
    type_hash,
    *allocator,
    &type_hash_c_str);
  if (RCUTILS_RET_BAD_ALLOC == stringify_ret) {
    RMW_SET_ERROR_MSG("Failed to allocate type_hash_c_str.");
    return nullptr;
  }
  auto free_type_hash_c_str = rcpputils::make_scope_exit(
    [&allocator, &type_hash_c_str]() {
      allocator->deallocate(type_hash_c_str, allocator->state);
    });

  std::size_t domain_id = node_info.domain_id_;
  auto entity = liveliness::Entity::make(
    z_info_zid(session),
    std::to_string(node_id),
    std::to_string(service_id),
    liveliness::EntityType::Service,
    std::move(node_info),
    liveliness::TopicInfo{
      std::move(domain_id),
      service_name,
      std::move(service_type),
      type_hash_c_str,
      std::move(adapted_qos_profile)}
  );
  if (entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the service %s.",
      service_name);
    return nullptr;
  }

  auto service_data = std::shared_ptr<ServiceData>(
    new ServiceData{
      node,
      std::move(entity),
      request_members,
      response_members,
      std::move(request_type_support),
      std::move(response_type_support)
    });

  // TODO(Yadunund): Instead of passing a rawptr, rely on capturing weak_ptr<ServiceData>
  // in the closure callback once we switch to zenoh-cpp.
  z_owned_closure_query_t callback = z_closure(service_data_handler, nullptr, service_data.get());
  service_data->keyexpr_ =
    z_keyexpr_new(service_data->entity_->topic_info().value().topic_keyexpr_.c_str());
  auto free_ros_keyexpr = rcpputils::make_scope_exit(
    [service_data]() {
      z_drop(z_move(service_data->keyexpr_));
    });
  if (!z_check(z_loan(service_data->keyexpr_))) {
    RMW_SET_ERROR_MSG("unable to create zenoh keyexpr.");
    return nullptr;
  }
  // Configure the queryable to process complete queries.
  z_queryable_options_t qable_options = z_queryable_options_default();
  qable_options.complete = true;
  service_data->qable_ = z_declare_queryable(
    session,
    z_loan(service_data->keyexpr_),
    z_move(callback),
    &qable_options);
  auto undeclare_z_queryable = rcpputils::make_scope_exit(
    [service_data]() {
      z_undeclare_queryable(z_move(service_data->qable_));
    });
  if (!z_check(service_data->qable_)) {
    RMW_SET_ERROR_MSG("unable to create zenoh queryable");
    return nullptr;
  }

  service_data->token_ = zc_liveliness_declare_token(
    session,
    z_keyexpr(service_data->entity_->liveliness_keyexpr().c_str()),
    NULL
  );
  auto free_token = rcpputils::make_scope_exit(
    [service_data]() {
      z_drop(z_move(service_data->token_));
    });
  if (!z_check(service_data->token_)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the service.");
    return nullptr;
  }

  free_ros_keyexpr.cancel();
  undeclare_z_queryable.cancel();
  free_token.cancel();

  return service_data;
}

///=============================================================================
ServiceData::ServiceData(
  const rmw_node_t * rmw_node,
  std::shared_ptr<liveliness::Entity> entity,
  const void * request_type_support_impl,
  const void * response_type_support_impl,
  std::unique_ptr<RequestTypeSupport> request_type_support,
  std::unique_ptr<ResponseTypeSupport> response_type_support)
: rmw_node_(rmw_node),
  entity_(std::move(entity)),
  request_type_support_impl_(request_type_support_impl),
  response_type_support_impl_(response_type_support_impl),
  request_type_support_(std::move(request_type_support)),
  response_type_support_(std::move(response_type_support)),
  wait_set_data_(nullptr),
  is_shutdown_(false)
{
  // Do nothing.
}

///=============================================================================
liveliness::TopicInfo ServiceData::topic_info() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->topic_info().value();
}

///=============================================================================
bool ServiceData::liveliness_is_valid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return zc_liveliness_token_check(&token_);
}

///=============================================================================
void ServiceData::add_new_query(std::unique_ptr<ZenohQuery> query)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Request from client will be ignored since the service is shutdown."
    );
    return;
  }
  const rmw_qos_profile_t adapted_qos_profile =
    entity_->topic_info().value().qos_;
  if (adapted_qos_profile.history != RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
    query_queue_.size() >= adapted_qos_profile.depth)
  {
    // Log warning if message is discarded due to hitting the queue depth
    z_owned_str_t keystr = z_keyexpr_to_string(z_loan(keyexpr_));
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Query queue depth of %ld reached, discarding oldest Query "
      "for service for %s",
      adapted_qos_profile.depth,
      z_loan(keystr));
    z_drop(z_move(keystr));
    query_queue_.pop_front();
  }
  query_queue_.emplace_back(std::move(query));

  // Since we added new data, trigger user callback and guard condition if they are available
  data_callback_mgr_.trigger_callback();
  if (wait_set_data_ != nullptr) {
    std::lock_guard<std::mutex> wait_set_lock(wait_set_data_->condition_mutex);
    wait_set_data_->triggered = true;
    wait_set_data_->condition_variable.notify_one();
  }
}

///=============================================================================
rmw_ret_t ServiceData::take_request(
  rmw_service_info_t * request_header,
  void * ros_request,
  bool * taken)
{
  std::lock_guard<std::mutex> lock(mutex_);
  *taken = false;

  if (is_shutdown_ || query_queue_.empty()) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  std::unique_ptr<ZenohQuery> query = std::move(query_queue_.front());
  query_queue_.pop_front();
  const z_query_t loaned_query = query->get_query();

  // DESERIALIZE MESSAGE ========================================================
  z_value_t payload_value = z_query_value(&loaned_query);

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(const_cast<uint8_t *>(payload_value.payload.start)),
    payload_value.payload.len);

  // Object that serializes the data
  Cdr deser(fastbuffer);
  if (!request_type_support_->deserialize_ros_message(
      deser.get_cdr(),
      ros_request,
      request_type_support_impl_))
  {
    RMW_SET_ERROR_MSG("could not deserialize ROS message");
    return RMW_RET_ERROR;
  }

  // Fill in the request header.
  // Get the sequence_number out of the attachment
  z_attachment_t attachment = z_query_attachment(&loaned_query);
  request_header->request_id.sequence_number =
    get_int64_from_attachment(&attachment, "sequence_number");
  if (request_header->request_id.sequence_number < 0) {
    RMW_SET_ERROR_MSG("Failed to get sequence_number from client call attachment");
    return RMW_RET_ERROR;
  }
  request_header->source_timestamp = get_int64_from_attachment(
    &attachment,
    "source_timestamp");
  if (request_header->source_timestamp < 0) {
    RMW_SET_ERROR_MSG("Failed to get source_timestamp from client call attachment");
    return RMW_RET_ERROR;
  }
  if (!get_gid_from_attachment(
      &attachment,
      request_header->request_id.writer_guid))
  {
    RMW_SET_ERROR_MSG("Could not get client GID from attachment");
    return RMW_RET_ERROR;
  }
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  request_header->received_timestamp = now_ns.count();

  // Add this query to the map, so that rmw_send_response can quickly look it up later.
  const size_t hash = rmw_zenoh_cpp::hash_gid(request_header->request_id.writer_guid);
  std::unordered_map<size_t, SequenceToQuery>::iterator it = sequence_to_query_map_.find(hash);
  if (it == sequence_to_query_map_.end()) {
    SequenceToQuery stq;
    sequence_to_query_map_.insert(std::make_pair(hash, std::move(stq)));
    it = sequence_to_query_map_.find(hash);
  } else {
    // Client already in the map
    if (it->second.find(request_header->request_id.sequence_number) != it->second.end()) {
      RMW_SET_ERROR_MSG("duplicate sequence number in the map");
      return RMW_RET_ERROR;
    }
  }

  it->second.insert(std::make_pair(request_header->request_id.sequence_number, std::move(query)));
  *taken = true;

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t ServiceData::send_response(
  rmw_request_id_t * request_id,
  void * ros_response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Unable to send response as the service is shutdown."
    );
    return RMW_RET_OK;
  }
  // Create the queryable payload
  const size_t hash = hash_gid(request_id->writer_guid);
  std::unordered_map<size_t, SequenceToQuery>::iterator it = sequence_to_query_map_.find(hash);
  if (it == sequence_to_query_map_.end()) {
    // If there is no data associated with this request, the higher layers of
    // ROS 2 seem to expect that we just silently return with no work.
    return RMW_RET_OK;
  }
  SequenceToQuery::iterator query_it = it->second.find(request_id->sequence_number);
  if (query_it == it->second.end()) {
    // If there is no data associated with this request, the higher layers of
    // ROS 2 seem to expect that we just silently return with no work.
    return RMW_RET_OK;
  }
  std::unique_ptr<ZenohQuery> query = std::move(query_it->second);
  it->second.erase(query_it);
  if (sequence_to_query_map_[hash].size() == 0) {
    sequence_to_query_map_.erase(hash);
  }

  rcutils_allocator_t * allocator = &(rmw_node_->context->options.allocator);

  size_t max_data_length = (
    response_type_support_->get_estimated_serialized_size(
      ros_response, response_type_support_impl_));

  // Init serialized message byte array
  char * response_bytes = static_cast<char *>(allocator->allocate(
      max_data_length,
      allocator->state));
  if (!response_bytes) {
    RMW_SET_ERROR_MSG("failed to allocate response message bytes");
    return RMW_RET_ERROR;
  }
  auto free_response_bytes = rcpputils::make_scope_exit(
    [response_bytes, allocator]() {
      allocator->deallocate(response_bytes, allocator->state);
    });

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(response_bytes, max_data_length);

  // Object that serializes the data
  rmw_zenoh_cpp::Cdr ser(fastbuffer);
  if (!response_type_support_->serialize_ros_message(
      ros_response,
      ser.get_cdr(),
      response_type_support_impl_))
  {
    return RMW_RET_ERROR;
  }

  size_t data_length = ser.get_serialized_data_length();
  const z_query_t loaned_query = query->get_query();
  z_query_reply_options_t options = z_query_reply_options_default();
  z_owned_bytes_map_t map = rmw_zenoh_cpp::create_map_and_set_sequence_num(
    request_id->sequence_number,
    [request_id](z_owned_bytes_map_t * map, const char * key)
    {
      z_bytes_t gid_bytes;
      gid_bytes.len = RMW_GID_STORAGE_SIZE;
      gid_bytes.start = request_id->writer_guid;
      z_bytes_map_insert_by_copy(map, z_bytes_new(key), gid_bytes);
    });
  if (!z_check(map)) {
    // create_map_and_set_sequence_num already set the error
    return RMW_RET_ERROR;
  }
  auto free_attachment_map = rcpputils::make_scope_exit(
    [&map]() {
      z_bytes_map_drop(z_move(map));
    });
  options.attachment = z_bytes_map_as_attachment(&map);

  z_query_reply(
    &loaned_query, z_loan(keyexpr_), reinterpret_cast<const uint8_t *>(
      response_bytes), data_length, &options);

  return RMW_RET_OK;
}

///=============================================================================
ServiceData::~ServiceData()
{
  const rmw_ret_t ret = this->shutdown();
  if (ret != RMW_RET_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Error destructing service /%s.",
      entity_->topic_info().value().name_.c_str()
    );
  }
}

//==============================================================================
void ServiceData::set_on_new_request_callback(
  rmw_event_callback_t callback,
  const void * user_data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_callback_mgr_.set_callback(user_data, std::move(callback));
}

///=============================================================================
bool ServiceData::queue_has_data_and_attach_condition_if_not(
  rmw_wait_set_data_t * wait_set_data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!query_queue_.empty()) {
    return true;
  }
  wait_set_data_ = wait_set_data;

  return false;
}

///=============================================================================
bool ServiceData::detach_condition_and_queue_is_empty()
{
  std::lock_guard<std::mutex> lock(mutex_);
  wait_set_data_ = nullptr;

  return query_queue_.empty();
}

///=============================================================================
rmw_ret_t ServiceData::shutdown()
{
  rmw_ret_t ret = RMW_RET_OK;
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return ret;
  }

  // Unregister this node from the ROS graph.
  if (zc_liveliness_token_check(&token_)) {
    zc_liveliness_undeclare_token(z_move(token_));
  }
  if (z_check(z_loan(keyexpr_))) {
    z_drop(z_move(keyexpr_));
  }
  if (z_check(qable_)) {
    z_undeclare_queryable(z_move(qable_));
  }

  is_shutdown_ = true;
  return RMW_RET_OK;
}

///=============================================================================
bool ServiceData::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}
}  // namespace rmw_zenoh_cpp
