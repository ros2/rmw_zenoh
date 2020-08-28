#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"

#include <rmw/validate_full_topic_name.h>
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
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "[rmw_create_service] %s", service_name);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr
  );

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
  int * validation_result = static_cast<int *>(allocator->allocate(sizeof(int), allocator->state));

  rmw_validate_full_topic_name(service_name, validation_result, nullptr);

  if (*validation_result == RMW_TOPIC_VALID
      || qos_profile->avoid_ros_namespace_conventions) {
    allocator->deallocate(validation_result, allocator->state);
  } else {
    RMW_SET_ERROR_MSG("service name is malformed!");
    allocator->deallocate(validation_result, allocator->state);
    return nullptr;
  }

  // OBTAIN TYPESUPPORT ========================================================
  const rosidl_service_type_support_t * type_support = get_service_typesupport_handle(
    type_supports, RMW_ZENOH_CPP_TYPESUPPORT_C
  );

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
    allocator->allocate(sizeof(rmw_service_t), allocator->state)
  );
  if (!service) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_service_t");
    allocator->deallocate(service, allocator->state);
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
    allocator->allocate(sizeof(rmw_service_data_t), allocator->state)
  );
  if (!service->data) {
    RMW_SET_ERROR_MSG("failed to allocate service data");
    allocator->deallocate(service->data, allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  // CREATE SERVICE MEMBERS ====================================================
  // Get typed pointer to implementation specific subscription data struct
  auto * service_data = static_cast<rmw_service_data_t *>(service->data);

  // Obtain Zenoh session and create Zenoh resource for response messages
  ZNSession * s = node->context->impl->session;
  service_data->zn_session_ = s;

  // Obtain qualified request-response topics
  std::string zn_request_topic_key(service->service_name);
  service_data->zn_request_topic_key_ = rcutils_strdup(
    (zn_request_topic_key + "/request").c_str(), *allocator
  );
  if (!service_data->zn_request_topic_key_) {
    RMW_SET_ERROR_MSG("failed to allocate zenoh request topic key");
    allocator->deallocate(service->data, allocator->state);

    allocator->deallocate(const_cast<char *>(service->service_name), allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  std::string zn_response_topic_key(service->service_name);
  service_data->zn_response_topic_key_ = rcutils_strdup(
    (zn_response_topic_key + "/response").c_str(), *allocator
  );
  if (!service_data->zn_response_topic_key_) {
    RMW_SET_ERROR_MSG("failed to allocate zenoh response topic key");
    allocator->deallocate(const_cast<char *>(service_data->zn_request_topic_key_), allocator->state);
    allocator->deallocate(service->data, allocator->state);

    allocator->deallocate(const_cast<char *>(service->service_name), allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  // NOTE(CH3): This topic ID only unique WITHIN this process!
  //
  // Another topic on another process might clash with the ID on this process, even within the
  // same Zenoh network! It is not a UUID!!
  service_data->zn_response_topic_id_ = zn_declare_resource(s,
                                                            service_data->zn_response_topic_key_);

  // Init type support callbacks
  auto service_members = static_cast<const service_type_support_callbacks_t *>(type_support->data);
  auto request_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->request_members_->data
  );
  auto response_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->response_members_->data
  );

  service_data->typesupport_identifier_ = type_support->typesupport_identifier;
  service_data->request_type_support_impl_ = request_members;
  service_data->response_type_support_impl_ = response_members;

  // Allocate and in-place assign new typesupport instances
  service_data->request_type_support_ = static_cast<rmw_zenoh_cpp::RequestTypeSupport *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::RequestTypeSupport), allocator->state)
  );
  new(service_data->request_type_support_) rmw_zenoh_cpp::RequestTypeSupport(service_members);
  if (!service_data->request_type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate RequestTypeSupport");
    allocator->deallocate(service_data->request_type_support_, allocator->state);
    allocator->deallocate(service->data, allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  service_data->response_type_support_ = static_cast<rmw_zenoh_cpp::ResponseTypeSupport *>(
    allocator->allocate(sizeof(rmw_zenoh_cpp::ResponseTypeSupport), allocator->state)
  );
  new(service_data->response_type_support_) rmw_zenoh_cpp::ResponseTypeSupport(service_members);
  if (!service_data->response_type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate ResponseTypeSupport");
    allocator->deallocate(service_data->request_type_support_, allocator->state);
    allocator->deallocate(service_data->response_type_support_, allocator->state);
    allocator->deallocate(service->data, allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  // // Assign node pointer
  service_data->node_ = node;

  // Init Zenoh subscriber for request messages
  service_data->zn_request_subscriber_ = zn_declare_subscriber(
    service_data->zn_session_,
    service_data->zn_request_topic_key_,
    zn_subinfo_default(),  // NOTE(CH3): Default for now
    service_data->zn_request_sub_callback
  );

  // Init Zenoh queryable for availability checking
  service_data->zn_queryable_ = zn_declare_queryable(
    s, service->service_name, EVAL, [](ZNQuery * query){
      const zn_string * resource = zn_query_res_name(query);
      const zn_string * predicate = zn_query_predicate(query);

      std::string key(resource->val, resource->len);
      std::string response("available");  // NOTE(CH3): The contents actually don't matter...

      zn_send_reply(query, key.c_str(), (const unsigned char *)response.c_str(), response.length());
    }
  );
  if (service_data->zn_queryable_ == 0) {
    RMW_SET_ERROR_MSG("failed to create availability queryable for service");

    zn_undeclare_subscriber(service_data->zn_request_subscriber_);

    allocator->deallocate(service_data->request_type_support_, allocator->state);
    allocator->deallocate(service_data->response_type_support_, allocator->state);
    allocator->deallocate(service->data, allocator->state);
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
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "[rmw_destroy_service] %s", service->service_name);

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

  // CLEANUP ===================================================================
  auto service_data = static_cast<rmw_service_data_t *>(service->data);

  zn_undeclare_subscriber(service_data->zn_request_subscriber_);
  zn_undeclare_queryable(service_data->zn_queryable_);

  allocator->deallocate(service_data->request_type_support_,
                        allocator->state);
  allocator->deallocate(service_data->response_type_support_,
                        allocator->state);
  allocator->deallocate(service->data, allocator->state);
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
  // RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take_request");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );

  // OBTAIN SUBSCRIPTION MEMBERS ===============================================
  const char * service_name = service->service_name;
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, RMW_RET_INVALID_ARGUMENT);

  rmw_service_data_t * service_data = static_cast<rmw_service_data_t *>(service->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_ERROR);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &service_data->node_->context->options.allocator;

  // RETRIEVE SERIALIZED MESSAGE ===============================================
  std::string key(service_data->zn_request_topic_key_);

  if (service_data->zn_request_messages_.find(key) == service_data->zn_request_messages_.end()) {
    // NOTE(CH3): It is correct to be returning RMW_RET_OK. The information that the message
    // was not found is encoded in the fact that the taken-out parameter is still False.
    //
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "[rmw_take_request] Request found: %s", key.c_str());

  auto request_bytes = service_data->zn_request_messages_[key];

  // RETRIEVE METADATA =========================================================
  //
  // TODO(CH3): Refactor this into its own modular set of functions eventually to make adding
  // more metadata convenient
  size_t meta_length = sizeof(rmw_client_data_t::sequence_id);

  // Use metadata
  memcpy(&request_header->request_id.sequence_number,
         &request_bytes.back() + 1 - meta_length,
         meta_length);

  // DESERIALIZE MESSAGE =======================================================
  size_t data_length = request_bytes.size() - meta_length;

  unsigned char * cdr_buffer = static_cast<unsigned char *>(
    allocator->allocate(data_length, allocator->state)
  );
  memcpy(cdr_buffer, &request_bytes.front(), data_length);

  // Remove stored message after successful retrieval
  service_data->zn_request_messages_.erase(key);

  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(cdr_buffer),
    data_length
  );  // Object that manages the raw buffer.

  eprosima::fastcdr::Cdr deser(fastbuffer,
                               eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                               eprosima::fastcdr::Cdr::DDS_CDR);  // Object that serializes the data.
  if (!service_data->request_type_support_->deserializeROSmessage(
    deser, ros_request, service_data->request_type_support_impl_)
  ) {
    RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "COULD NOT DESERIALIZE REQUEST MESSAGE");
    return RMW_RET_ERROR;
  }

  *taken = true;
  return RMW_RET_OK;
}

/// SEND SERVICE RESPONSE ======================================================
// Serialize and publish a ROS response message using Zenoh.
rmw_ret_t
rmw_send_response(const rmw_service_t * service,
                  rmw_request_id_t * request_header,  // In parameter
                  void * ros_response)
{
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp",
                         "[rmw_send_response] %s (%ld)",
                         service->service_name,
                         static_cast<rmw_service_data_t *>(service->data)->zn_response_topic_id_);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );

  auto service_data = static_cast<rmw_service_data_t *>(service->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_ERROR);

  // ASSIGN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator =
    &static_cast<rmw_service_data_t *>(service->data)->node_->context->options.allocator;

  // SERIALIZE DATA ============================================================
  size_t max_data_length = (
    static_cast<rmw_service_data_t *>(service->data)
      ->response_type_support_->getEstimatedSerializedSize(ros_response)
  );

  // Account for metadata
  max_data_length += sizeof(request_header->sequence_number);

  // Init serialized message byte array
  char * response_bytes = static_cast<char *>(
    allocator->allocate(max_data_length, allocator->state)
  );

  eprosima::fastcdr::FastBuffer fastbuffer(
    response_bytes,
    max_data_length
  );  // Object that manages the raw buffer.

  eprosima::fastcdr::Cdr ser(fastbuffer,
                             eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                             eprosima::fastcdr::Cdr::DDS_CDR);  // Object that serializes the data.
  if (!service_data->response_type_support_->serializeROSmessage(
    ros_response, ser, service_data->response_type_support_impl_)
  ) {
    allocator->deallocate(response_bytes, allocator->state);
    return RMW_RET_ERROR;
  }

  size_t data_length = ser.getSerializedDataLength();

  // ADD METADATA ==============================================================
  // TODO(CH3): Again, refactor this into a modular set of functions eventually
  size_t meta_length = sizeof(request_header->sequence_number);
  memcpy(
    &response_bytes[data_length], reinterpret_cast<char *>(&request_header->sequence_number), meta_length
  );

  // PUBLISH ON ZENOH MIDDLEWARE LAYER =========================================
  zn_write_wrid(service_data->zn_session_,
                service_data->zn_response_topic_id_,
                response_bytes,
                data_length + meta_length);

  allocator->deallocate(response_bytes, allocator->state);
  return RMW_RET_OK;
}

} // extern "C"
