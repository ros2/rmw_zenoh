#include <string>

#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"

#include "rmw/validate_full_topic_name.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/rmw.h"

#include "rmw_zenoh_cpp/rmw_context_impl.hpp"
#include "rmw_zenoh_cpp/identifier.hpp"

#include "impl/type_support_common.hpp"
#include "impl/service_impl.hpp"
#include "impl/client_impl.hpp"

extern "C"
{
/// CREATE SERVICE SERVER ======================================================
// Create and return an rmw service server
rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[rmw_create_service] %s with queue of depth %ld",
    service_name,
    qos_profile->depth);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr);

  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  if (strlen(service_name) == 0) {
    RMW_SET_ERROR_MSG("service name is empty string");
    return nullptr;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // VALIDATE SERVICE NAME =====================================================
  int validation_result;

  if (rmw_validate_full_topic_name(service_name, &validation_result, nullptr) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("rmw_validate_full_topic_name failed");
    return nullptr;
  }

  if (validation_result != RMW_TOPIC_VALID && !qos_profile->avoid_ros_namespace_conventions) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("service name is malformed: %s", service_name);
    return nullptr;
  }

  // OBTAIN TYPESUPPORT ========================================================
  const rosidl_service_type_support_t * type_support = get_service_typesupport_handle(
    type_supports,
    RMW_ZENOH_CPP_TYPESUPPORT_C);

  if (!type_support) {
    type_support = get_service_typesupport_handle(type_supports, RMW_ZENOH_CPP_TYPESUPPORT_CPP);
    if (!type_support) {
      RCUTILS_LOG_INFO("%s", service_name);
      RMW_SET_ERROR_MSG("type support not from this implementation");
      return nullptr;
    }
  }

  // CREATE SERVICE ============================================================
  rmw_service_t * service = static_cast<rmw_service_t *>(
    allocator->allocate(sizeof(rmw_service_t),
    allocator->state));
  if (!service) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_service_t");
    return nullptr;
  }

  // Populate common members
  service->implementation_identifier = eclipse_zenoh_identifier;  // const char * assignment

  service->service_name = rcutils_strdup(service_name, *allocator);  // const char * assignment
  if (!service->service_name) {
    RMW_SET_ERROR_MSG("failed to allocate service name");
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  service->data = static_cast<rmw_service_data_t *>(
    allocator->allocate(sizeof(rmw_service_data_t),
    allocator->state));
  new(service->data) rmw_service_data_t();
  if (!service->data) {
    RMW_SET_ERROR_MSG("failed to allocate service data");
    allocator->deallocate(const_cast<char *>(service->service_name), allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  // CREATE SERVICE MEMBERS ====================================================
  // Get typed pointer to implementation specific service data struct
  auto * service_data = static_cast<rmw_service_data_t *>(service->data);

  // Obtain Zenoh session and create Zenoh resource for response messages
  ZNSession * session = node->context->impl->session;
  service_data->zn_session_ = session;

  // Obtain qualified request-response topics
  std::string zn_topic_key(service->service_name);
  service_data->zn_request_topic_key_ = rcutils_strdup(
    (zn_topic_key + "/request").c_str(),
    *allocator);
  if (!service_data->zn_request_topic_key_) {
    RMW_SET_ERROR_MSG("failed to allocate zenoh request topic key");
    allocator->deallocate(service->data, allocator->state);

    allocator->deallocate(const_cast<char *>(service->service_name), allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  service_data->zn_response_topic_key_ = rcutils_strdup(
    (zn_topic_key + "/response").c_str(),
    *allocator);
  if (!service_data->zn_response_topic_key_) {
    RMW_SET_ERROR_MSG("failed to allocate zenoh response topic key");
    allocator->deallocate(
      const_cast<char *>(service_data->zn_request_topic_key_),
      allocator->state);
    allocator->deallocate(service->data, allocator->state);

    allocator->deallocate(const_cast<char *>(service->service_name), allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  // The topic ID must be unique within a single process, but separate processes can reuse IDs,
  // even in the same Zenoh network, because the ID is never transmitted over the wire.
  // Conversely, the ID used in two communicating processes cannot be used to determine if they are
  // using the same topic or not.
  service_data->zn_response_topic_id_ = zn_declare_resource(
    session,
    service_data->zn_response_topic_key_);

  // INSERT TYPE SUPPORT =======================================================
  // Init type support callbacks
  auto service_members = static_cast<const service_type_support_callbacks_t *>(type_support->data);
  auto request_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->request_members_->data);
  auto response_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->response_members_->data);

  service_data->typesupport_identifier_ = type_support->typesupport_identifier;
  service_data->request_type_support_impl_ = request_members;
  service_data->response_type_support_impl_ = response_members;

  // Allocate and in-place assign new typesupport instances
  service_data->request_type_support_ = static_cast<rmw_zenoh_cpp::RequestTypeSupport *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::RequestTypeSupport),
    allocator->state));
  new(service_data->request_type_support_) rmw_zenoh_cpp::RequestTypeSupport(service_members);
  if (!service_data->request_type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate RequestTypeSupport");
    allocator->deallocate(
      const_cast<char *>(service_data->zn_request_topic_key_),
      allocator->state);
    allocator->deallocate(
      const_cast<char *>(service_data->zn_response_topic_key_),
      allocator->state);
    allocator->deallocate(service->data, allocator->state);

    allocator->deallocate(const_cast<char *>(service->service_name), allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  service_data->response_type_support_ = static_cast<rmw_zenoh_cpp::ResponseTypeSupport *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::ResponseTypeSupport),
    allocator->state));
  new(service_data->response_type_support_) rmw_zenoh_cpp::ResponseTypeSupport(service_members);
  if (!service_data->response_type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate ResponseTypeSupport");
    allocator->deallocate(
      const_cast<char *>(service_data->zn_request_topic_key_),
      allocator->state);
    allocator->deallocate(
      const_cast<char *>(service_data->zn_response_topic_key_),
      allocator->state);
    allocator->deallocate(service_data->request_type_support_, allocator->state);
    allocator->deallocate(service->data, allocator->state);

    allocator->deallocate(const_cast<char *>(service->service_name), allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  // CONFIGURE SERVICE =========================================================
  // Assign node pointer
  service_data->node_ = node;

  // Assign and increment unique service ID atomically
  service_data->service_id_ =
    rmw_service_data_t::service_id_counter.fetch_add(1, std::memory_order_relaxed);

  // Configure request message queue
  service_data->queue_depth_ = qos_profile->depth;

  // ADD SERVICE DATA TO TOPIC MAP =============================================
  // This will allow us to access the service data structs for this Zenoh topic key expression
  std::string key(service_data->zn_request_topic_key_);
  auto map_iter = rmw_service_data_t::zn_topic_to_service_data.find(key);

  if (map_iter == rmw_service_data_t::zn_topic_to_service_data.end()) {
    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[rmw_create_service] New request topic detected: %s",
      service_data->zn_request_topic_key_);

    // If no elements for this Zenoh topic key expression exists, add it in
    std::vector<rmw_service_data_t *> service_data_vec{service_data};
    rmw_service_data_t::zn_topic_to_service_data[key] = service_data_vec;

    // We initialise subscribers ONCE (otherwise we'll get duplicate messages)
    // The topic name will be the same for any duplicate subscribers, so it is ok
    service_data->zn_request_subscriber_ = zn_declare_subscriber(
      service_data->zn_session_,
      service_data->zn_request_topic_key_,
      zn_subinfo_default(),  // NOTE(CH3): Default for now
      service_data->zn_request_sub_callback);

    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[rmw_create_service] Zenoh subscriber declared for %s",
      service_data->zn_request_topic_key_);
  } else {
    // Otherwise, append to the vector
    map_iter->second.push_back(service_data);
  }

  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[rmw_create_service] Service for %s (ID: %ld) added to topic map",
    service->service_name,
    service_data->service_id_);

  // DECLARE SERVICE IS AVAILABLE ==============================================
  service_data->zn_queryable_ = zn_declare_queryable(
    session,
    service->service_name,
    STORAGE,
    rmw_service_data_t::zn_service_availability_queryable_callback);

  if (service_data->zn_queryable_ == 0) {
    RMW_SET_ERROR_MSG("failed to create availability queryable for service");

    // Delete the service data pointer in the Zenoh topic to service data map
    for (auto it = map_iter->second.begin(); it != map_iter->second.end(); ++it) {
      if ((*it)->service_id_ == service_data->service_id_) {
        map_iter->second.erase(it);
        break;
      }
    }

    // Delete the map element if no other client data pointers exist
    // (That is, when no other services are listening to the Zenoh request topic)
    if (map_iter->second.empty()) {
      zn_undeclare_subscriber(service_data->zn_request_subscriber_);
      rmw_service_data_t::zn_topic_to_service_data.erase(map_iter);
    }

    allocator->deallocate(const_cast<char *>(service_data->zn_request_topic_key_), allocator->state);
    allocator->deallocate(const_cast<char *>(service_data->zn_response_topic_key_), allocator->state);
    allocator->deallocate(service_data->request_type_support_, allocator->state);
    allocator->deallocate(service_data->response_type_support_, allocator->state);
    allocator->deallocate(service->data, allocator->state);

    allocator->deallocate(const_cast<char *>(service->service_name), allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  return service;
}

/// DESTROY SERVICE SERVER =====================================================
// Destroy and deallocate an RMW service server
rmw_ret_t
rmw_destroy_service(rmw_node_t * node, rmw_service_t * service)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "[rmw_destroy_service] %s", service->service_name);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // OBTAIN SERVICE MEMBERS ====================================================
  auto * service_data = static_cast<rmw_service_data_t *>(service->data);

  // DELETE SERVICE DATA IN TOPIC MAP ==========================================
  std::string key(service_data->zn_request_topic_key_);
  auto map_iter = rmw_service_data_t::zn_topic_to_service_data.find(key);

  if (map_iter == rmw_service_data_t::zn_topic_to_service_data.end()) {
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "service not found in Zenoh topic to service data map! %s",
      service_data->zn_request_topic_key_);
  } else {
    // Delete the subscription data pointer in the Zenoh topic to service data map
    for (auto it = map_iter->second.begin(); it != map_iter->second.end(); ++it) {
      if ((*it)->service_id_ == service_data->service_id_) {
        map_iter->second.erase(it);
        break;
      }
    }

    // Delete the map element if no other service data pointers exist
    // (That is, when no other services are listening to the Zenoh topic)
    if (map_iter->second.empty()) {
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp",
        "[rmw_destroy_service] No more services listening to %s",
        service_data->zn_request_topic_key_);

      // Only when there are no more active RMW services listening to this Zenoh topic, do we
      // undeclare the subscriber on Zenoh's end (which means no more Zenoh callbacks will trigger
      // on this topic)
      zn_undeclare_subscriber(service_data->zn_request_subscriber_);
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp",
        "[rmw_destroy_service] Zenoh subcriber undeclared for %s",
        service_data->zn_request_topic_key_);

      rmw_service_data_t::zn_topic_to_service_data.erase(map_iter);
    }

    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[rmw_destroy_service] Service for %s (ID: %ld) removed from topic map",
      service_data->zn_request_topic_key_,
      service_data->service_id_);
  }

  // CLEANUP ===================================================================
  zn_undeclare_queryable(service_data->zn_queryable_);

  allocator->deallocate(const_cast<char *>(service_data->zn_request_topic_key_), allocator->state);
  allocator->deallocate(const_cast<char *>(service_data->zn_response_topic_key_), allocator->state);
  allocator->deallocate(service_data->request_type_support_, allocator->state);
  allocator->deallocate(service_data->response_type_support_, allocator->state);
  allocator->deallocate(service->data, allocator->state);

  allocator->deallocate(const_cast<char *>(service->service_name), allocator->state);
  allocator->deallocate(service, allocator->state);

  return RMW_RET_OK;
}

/// TAKE REQUEST MESSAGE =======================================================
// Take request message out of the request message queue
rmw_ret_t
rmw_take_request(
  const rmw_service_t * service,
  rmw_service_info_t * request_header,  // Out parameter
  void * ros_request,
  bool * taken)
{
  *taken = false;
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_take_request");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_FOR_NULL_WITH_MSG(
    service->service_name, "service has no service name", RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    service->data, "service implementation pointer is null", RMW_RET_INVALID_ARGUMENT);

  // OBTAIN SERVICE MEMBERS ====================================================
  auto * service_data = static_cast<rmw_service_data_t *>(service->data);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &service_data->node_->context->options.allocator;

  // RETRIEVE SERIALIZED MESSAGE ===============================================
  std::unique_lock<std::mutex> lock(service_data->request_queue_mutex_);

  if (service_data->zn_request_message_queue_.empty()) {
    // NOTE(CH3): It is correct to be returning RMW_RET_OK. The information that the message
    // was not found is encoded in the fact that the taken-out parameter is still False.
    //
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }

  // NOTE(CH3): Potential place to handle "QoS" (e.g. could pop from back so it is LIFO)
  auto request_bytes_ptr = service_data->zn_request_message_queue_.back();
  service_data->zn_request_message_queue_.pop_back();

  service_data->request_queue_mutex_.unlock();

  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[rmw_take] Request found: %s",
    service_data->zn_request_topic_key_);

  // RETRIEVE METADATA =========================================================
  //
  // TODO(CH3): Refactor this into its own modular set of functions eventually to make adding
  // more metadata convenient
  size_t meta_length = sizeof(std::int64_t); // Internal type of the atomic sequence ID

  // Use metadata
  memcpy(
    &request_header->request_id.sequence_number,
    &request_bytes_ptr->back() + 1 - meta_length,
    meta_length);

  // DESERIALIZE MESSAGE =======================================================
  size_t data_length = request_bytes_ptr->size() - meta_length;

  unsigned char * cdr_buffer = static_cast<unsigned char *>(
    allocator->allocate(data_length,
    allocator->state));
  memcpy(cdr_buffer, &request_bytes_ptr->front(), data_length);

  // Object that manages the raw buffer.
  eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char *>(cdr_buffer), data_length);

  // Object that deserializes the data
  eprosima::fastcdr::Cdr deser(
    fastbuffer,
    eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
    eprosima::fastcdr::Cdr::DDS_CDR);
  if (!service_data->request_type_support_->deserializeROSmessage(
      deser, ros_request, service_data->request_type_support_impl_)
  ) {
    RMW_SET_ERROR_MSG("could not deserialize ROS request message");
    return RMW_RET_ERROR;
  }

  *taken = true;
  allocator->deallocate(cdr_buffer, allocator->state);

  return RMW_RET_OK;
}

/// SEND SERVICE RESPONSE ======================================================
// Serialize and publish a ROS response message using Zenoh.
rmw_ret_t
rmw_send_response(const rmw_service_t * service,
                  rmw_request_id_t * request_header,  // In parameter
                  void * ros_response)
{
  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp", "[rmw_send_response] %s (%ld)",
    static_cast<rmw_service_data_t *>(service->data)->zn_response_topic_key_,
    static_cast<rmw_service_data_t *>(service->data)->zn_response_topic_id_);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_FOR_NULL_WITH_MSG(
    service->data,
    "service implementation pointer is null",
    RMW_RET_INVALID_ARGUMENT);

  // OBTAIN SERVICE MEMBERS ====================================================
  auto * service_data = static_cast<rmw_service_data_t *>(service->data);

  // ASSIGN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator =
    &(static_cast<rmw_service_data_t *>(service->data)->node_->context->options.allocator);

  // SERIALIZE DATA ============================================================
  size_t max_data_length = (
    static_cast<rmw_service_data_t *>(service->data)
    ->response_type_support_->getEstimatedSerializedSize(ros_response));

  // Account for metadata
  max_data_length += sizeof(request_header->sequence_number);

  // Init serialized message byte array
  char * response_bytes = static_cast<char *>(
    allocator->allocate(max_data_length,
    allocator->state));
  if (!response_bytes) {
    RMW_SET_ERROR_MSG("failed allocate response message bytes");
    allocator->deallocate(response_bytes, allocator->state);
    return RMW_RET_ERROR;
  }

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(response_bytes, max_data_length);

  // Object that serializes the data
  eprosima::fastcdr::Cdr ser(
    fastbuffer,
    eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
    eprosima::fastcdr::Cdr::DDS_CDR);
  if (!service_data->response_type_support_->serializeROSmessage(
      ros_response,
      ser,
      service_data->response_type_support_impl_))
    {
    allocator->deallocate(response_bytes, allocator->state);
    return RMW_RET_ERROR;
  }

  size_t data_length = ser.getSerializedDataLength();

  // ADD METADATA ==============================================================
  // TODO(CH3): Again, refactor this into a modular set of functions eventually
  size_t meta_length = sizeof(request_header->sequence_number);
  memcpy(
    &response_bytes[data_length],
    reinterpret_cast<char *>(&request_header->sequence_number),
    meta_length);

  // PUBLISH ON ZENOH MIDDLEWARE LAYER =========================================
  size_t wrid_ret = zn_write_wrid(
    service_data->zn_session_,
    service_data->zn_response_topic_id_,
    response_bytes,
    data_length + meta_length);

  allocator->deallocate(response_bytes, allocator->state);

  if (wrid_ret == 0) {
    return RMW_RET_OK;
  } else {
    RMW_SET_ERROR_MSG("zenoh failed to publish response");
    return RMW_RET_ERROR;
  }
}
}  // extern "C"
