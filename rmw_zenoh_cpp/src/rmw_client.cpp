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
#include "impl/client_impl.hpp"

extern "C"
{

/// CHECK IF SERVER IS AVAILABLE ===============================================
// Check if a service server is available for the given service client
rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * result)
{
  *result = false;

  RCUTILS_LOG_INFO_NAMED(
      "rmw_zenoh_cpp", "[rmw_service_server_is_available] %s", client->service_name);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );

  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(result, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_FOR_NULL_WITH_MSG(
      client->data, "client implementation pointer is null", RMW_RET_INVALID_ARGUMENT);

  // OBTAIN CLIENT MEMBERS =====================================================
  auto client_data = static_cast<rmw_client_data_t *>(client->data);

  // CHECK SERVER AVAILABILITY =================================================
  // Check if server is alive by querying its availability Zenoh queryable
  zn_query(client_data->zn_session_,
           client->service_name,
           "",  // NOTE(CH3): Maybe use this predicate we want to more things in the queryable
           zn_query_target_default(),
           zn_query_consolidation_default(),
           rmw_client_data_t::zn_service_availability_query_callback);

  std::string key(client->service_name);

  if (rmw_client_data_t::zn_availability_query_responses_.find(key)
      != rmw_client_data_t::zn_availability_query_responses_.end()) {
    rmw_client_data_t::zn_availability_query_responses_.erase(key);
    *result = true;
  }

  return RMW_RET_OK;
}

// CREATE SERVICE CLIENT =======================================================
// Create and return an rmw service client
rmw_client_t *
rmw_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "[rmw_create_client] %s", service_name);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(node,
                                   node->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return nullptr);

  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  if (strlen(service_name) == 0) {
    RMW_SET_ERROR_MSG("service name is empty string");
    return nullptr;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);

  // TODO(CH3): When we figure out how to spoof QoS, check for a 'configured' QoS to pass the final
  // test that is failing
  //
  // if (# SOME CHECK HERE) {
  //   RMW_SET_ERROR_MSG("expected configured QoS profile");
  //   return nullptr;
  // }

  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // VALIDATE SERVICE NAME =====================================================
  int * validation_result = static_cast<int *>(allocator->allocate(sizeof(int), allocator->state));
  if (!validation_result) {
    RMW_SET_ERROR_MSG("failed to allocate service name validation result storage pointer");
    return nullptr;
  }

  if (rmw_validate_full_topic_name(service_name, validation_result, nullptr) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("rmw_validate_full_topic_name failed");
    allocator->deallocate(validation_result, allocator->state);
    return nullptr;
  }

  if (*validation_result == RMW_TOPIC_VALID || qos_profile->avoid_ros_namespace_conventions) {
    allocator->deallocate(validation_result, allocator->state);
  } else {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("service name is malformed: %s", service_name);
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

  // CREATE CLIENT =============================================================
  rmw_client_t * client = static_cast<rmw_client_t *>(allocator->allocate(sizeof(rmw_client_t),
                                                                          allocator->state));
  if (!client) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_client_t");
    return nullptr;
  }

  // Populate common members
  client->implementation_identifier = eclipse_zenoh_identifier;

  client->service_name = rcutils_strdup(service_name, *allocator);
  if (!client->service_name) {
    RMW_SET_ERROR_MSG("failed to allocate service name for client");
    allocator->deallocate(client, allocator->state);
    return nullptr;
  }

  client->data = static_cast<rmw_client_data_t *>(allocator->allocate(sizeof(rmw_client_data_t),
                                                                      allocator->state));
  if (!client->data) {
    RMW_SET_ERROR_MSG("failed to allocate client data");
    allocator->deallocate(const_cast<char *>(client->service_name), allocator->state);
    allocator->deallocate(client, allocator->state);
    return nullptr;
  }

  // CREATE CLIENT MEMBERS =====================================================
  // Get typed pointer to implementation specific subscription data struct
  auto client_data = static_cast<rmw_client_data_t *>(client->data);

  // Obtain Zenoh session and create Zenoh resource for request messages
  ZNSession * s = node->context->impl->session;
  client_data->zn_session_ = s;

  // Obtain qualified request-response topics
  std::string zn_request_topic_key(client->service_name);
  client_data->zn_request_topic_key_ = rcutils_strdup(
    (zn_request_topic_key + "/request").c_str(), *allocator
  );
  if (!client_data->zn_request_topic_key_) {
    RMW_SET_ERROR_MSG("failed to allocate zenoh request topic key");
    allocator->deallocate(client->data, allocator->state);

    allocator->deallocate(const_cast<char *>(client->service_name), allocator->state);
    allocator->deallocate(client, allocator->state);
    return nullptr;
  }

  std::string zn_response_topic_key(client->service_name);
  client_data->zn_response_topic_key_ = rcutils_strdup(
    (zn_response_topic_key + "/response").c_str(), *allocator
  );
  if (!client_data->zn_response_topic_key_) {
    RMW_SET_ERROR_MSG("failed to allocate zenoh response topic key");
    allocator->deallocate(const_cast<char *>(client_data->zn_request_topic_key_), allocator->state);
    allocator->deallocate(client->data, allocator->state);

    allocator->deallocate(const_cast<char *>(client->service_name), allocator->state);
    allocator->deallocate(client, allocator->state);
    return nullptr;
  }

  // NOTE(CH3): This topic ID only unique WITHIN this process!
  //
  // Another topic on another process might clash with the ID on this process, even within the
  // same Zenoh network! It is not a UUID!!
  client_data->zn_request_topic_id_ = zn_declare_resource(s, client_data->zn_request_topic_key_);

  // Init type support callbacks
  auto service_members = static_cast<const service_type_support_callbacks_t *>(type_support->data);
  auto request_members = static_cast<const message_type_support_callbacks_t *>(
      service_members->request_members_->data);
  auto response_members = static_cast<const message_type_support_callbacks_t *>(
      service_members->response_members_->data);

  client_data->typesupport_identifier_ = type_support->typesupport_identifier;
  client_data->request_type_support_impl_ = request_members;
  client_data->response_type_support_impl_ = response_members;

  // Allocate and in-place assign new typesupport instances
  client_data->request_type_support_ = static_cast<rmw_zenoh_cpp::RequestTypeSupport *>(
      allocator->allocate(sizeof(rmw_zenoh_cpp::RequestTypeSupport), allocator->state));
  new(client_data->request_type_support_) rmw_zenoh_cpp::RequestTypeSupport(service_members);
  if (!client_data->request_type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate RequestTypeSupport");
    allocator->deallocate(const_cast<char *>(client_data->zn_request_topic_key_), allocator->state);
    allocator->deallocate(const_cast<char *>(client_data->zn_response_topic_key_), allocator->state);
    allocator->deallocate(client->data, allocator->state);

    allocator->deallocate(const_cast<char *>(client->service_name), allocator->state);
    allocator->deallocate(client, allocator->state);
    return nullptr;
  }

  client_data->response_type_support_ = static_cast<rmw_zenoh_cpp::ResponseTypeSupport *>(
      allocator->allocate(sizeof(rmw_zenoh_cpp::ResponseTypeSupport), allocator->state));
  new(client_data->response_type_support_) rmw_zenoh_cpp::ResponseTypeSupport(service_members);
  if (!client_data->response_type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate ResponseTypeSupport");
    allocator->deallocate(const_cast<char *>(client_data->zn_request_topic_key_), allocator->state);
    allocator->deallocate(const_cast<char *>(client_data->zn_response_topic_key_), allocator->state);
    allocator->deallocate(client_data->request_type_support_, allocator->state);
    allocator->deallocate(client->data, allocator->state);

    allocator->deallocate(const_cast<char *>(client->service_name), allocator->state);
    allocator->deallocate(client, allocator->state);
    return nullptr;
  }

  // // Assign node pointer
  client_data->node_ = node;

  // Init Zenoh subscriber for response messages
  client_data->zn_response_subscriber_ = zn_declare_subscriber(
      client_data->zn_session_,
      client_data->zn_response_topic_key_,
      zn_subinfo_default(),  // NOTE(CH3): Default for now
      client_data->zn_response_sub_callback);

  return client;
}

/// DESTROY SERVICE CLIENT =====================================================
// Destroy and deallocate an RMW service client
rmw_ret_t
rmw_destroy_client(rmw_node_t * node, rmw_client_t * client)
{
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "[rmw_destroy_client] %s", client->service_name);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(node,
                                   node->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(client,
                                   client->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // CLEANUP ===================================================================
  auto client_data = static_cast<rmw_client_data_t *>(client->data);

  zn_undeclare_subscriber(client_data->zn_response_subscriber_);

  allocator->deallocate(const_cast<char *>(client_data->zn_request_topic_key_), allocator->state);
  allocator->deallocate(const_cast<char *>(client_data->zn_response_topic_key_), allocator->state);
  allocator->deallocate(client_data->request_type_support_, allocator->state);
  allocator->deallocate(client_data->response_type_support_, allocator->state);
  allocator->deallocate(client->data, allocator->state);

  allocator->deallocate(const_cast<char *>(client->service_name), allocator->state);
  allocator->deallocate(client, allocator->state);

  return RMW_RET_OK;
}

/// SEND SERVICE REQUESST ======================================================
// Serialize and publish a ROS request message using Zenoh.
//
// sequence_id is an out parameter
rmw_ret_t
rmw_send_request(const rmw_client_t * client, const void * ros_request, int64_t * sequence_id)
{
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp",
                         "[rmw_send_request] %s (%ld)",
                         static_cast<rmw_client_data_t *>(client->data)->zn_request_topic_key_,
                         static_cast<rmw_client_data_t *>(client->data)->zn_request_topic_id_);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(sequence_id, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(client,
                                   client->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_ARGUMENT_FOR_NULL(client->data, RMW_RET_ERROR);
  auto client_data = static_cast<rmw_client_data_t *>(client->data);

  // ASSIGN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator =
    &static_cast<rmw_client_data_t *>(client->data)->node_->context->options.allocator;

  // SERIALIZE DATA ============================================================
  size_t max_data_length = (static_cast<rmw_client_data_t *>(client->data)
                              ->request_type_support_->getEstimatedSerializedSize(ros_request));

  // Account for metadata
  max_data_length += sizeof(rmw_client_data_t::sequence_id);

  // Init serialized message byte array
  char * request_bytes = static_cast<char *>(
      allocator->allocate(max_data_length, allocator->state));
  if (!request_bytes) {
    RMW_SET_ERROR_MSG("failed allocate request message bytes");
    allocator->deallocate(request_bytes, allocator->state);
    return RMW_RET_ERROR;
  }

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(request_bytes, max_data_length);

  // Object that serializes the data.
  eprosima::fastcdr::Cdr ser(
      fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);

  if (!client_data->request_type_support_->serializeROSmessage(
          ros_request, ser, client_data->request_type_support_impl_)) {
    RMW_SET_ERROR_MSG("failed serialize ROS request message");
    allocator->deallocate(request_bytes, allocator->state);
    return RMW_RET_ERROR;
  }

  size_t data_length = ser.getSerializedDataLength();

  // ADD METADATA ==============================================================
  //
  // TODO(CH3): Refactor this into its own modular set of functions eventually to make adding
  // more metadata convenient
  rmw_client_data_t::sequence_id++;
  *sequence_id = rmw_client_data_t::sequence_id;

  size_t meta_length = sizeof(rmw_client_data_t::sequence_id);
  memcpy(request_bytes + data_length,
         reinterpret_cast<char *>(&rmw_client_data_t::sequence_id),
         meta_length);

  // PUBLISH ON ZENOH MIDDLEWARE LAYER =========================================
  size_t wrid_ret = zn_write_wrid(client_data->zn_session_,
                                  client_data->zn_request_topic_id_,
                                  request_bytes,
                                  data_length + meta_length);

  allocator->deallocate(request_bytes, allocator->state);

  if (wrid_ret == 0) {
    return RMW_RET_OK;
  } else {
    RMW_SET_ERROR_MSG("zenoh failed to publish request");
    return RMW_RET_ERROR;
  }
}

/// TAKE RESPONSE MESSAGE ======================================================
// Take response message out of the response message queue
rmw_ret_t
rmw_take_response(
  const rmw_client_t * client,
  rmw_service_info_t * request_header,  // Out parameter
  void * ros_response,
  bool * taken)
{
  *taken = false;

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(client,
                                   client->implementation_identifier,
                                   eclipse_zenoh_identifier,
                                   return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_FOR_NULL_WITH_MSG(
      client->service_name, "client has no service name", RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
      client->data, "client implementation pointer is null", RMW_RET_INVALID_ARGUMENT);

  // OBTAIN CLIENT MEMBERS =====================================================
  auto client_data = static_cast<rmw_client_data_t *>(client->data);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &client_data->node_->context->options.allocator;

  // RETRIEVE SERIALIZED MESSAGE ===============================================
  std::string key(client_data->zn_response_topic_key_);

  if (client_data->zn_response_messages_.find(key) == client_data->zn_response_messages_.end()) {
    // NOTE(CH3): It is correct to be returning RMW_RET_OK. The information that the message
    // was not found is encoded in the fact that the taken out-parameter is still False.
    //
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "[rmw_take_response] Response found: %s", key.c_str());

  auto response_bytes = client_data->zn_response_messages_[key];

  // RETRIEVE METADATA =========================================================
  // TODO(CH3): Again, refactor this into a modular set of functions eventually
  size_t meta_length = sizeof(rmw_client_data_t::sequence_id);

  // Use metadata
  memcpy(&request_header->request_id.sequence_number,
         &response_bytes.back() + 1 - meta_length,
         meta_length);

  // DESERIALIZE MESSAGE =======================================================
  size_t data_length = response_bytes.size() - meta_length;

  unsigned char * cdr_buffer = static_cast<unsigned char *>(
      allocator->allocate(data_length, allocator->state));
  if (!cdr_buffer) {
    RMW_SET_ERROR_MSG("failed allocate response message bytes");
    allocator->deallocate(cdr_buffer, allocator->state);
    return RMW_RET_ERROR;
  }
  memcpy(cdr_buffer, &response_bytes.front(), data_length);

  // Remove stored message after successful retrieval
  client_data->zn_response_messages_.erase(key);

  // Object that manages the raw buffer.
  eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char *>(cdr_buffer), data_length);

  // Object that deserializes the data.
  eprosima::fastcdr::Cdr deser(fastbuffer,
                               eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                               eprosima::fastcdr::Cdr::DDS_CDR);

  if (!client_data->response_type_support_->deserializeROSmessage(
      deser, ros_response, client_data->response_type_support_impl_)) {
    RMW_SET_ERROR_MSG("failed deserialize ROS response message");
    return RMW_RET_ERROR;
  }

  *taken = true;
  allocator->deallocate(cdr_buffer, allocator->state);

  return RMW_RET_OK;
}

} // extern "C"
