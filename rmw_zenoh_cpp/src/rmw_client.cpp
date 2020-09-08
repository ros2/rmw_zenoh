#include <unistd.h>
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

  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[rmw_service_server_is_available] %s",
    client->service_name);

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
    client->data,
    "client implementation pointer is null",
    RMW_RET_INVALID_ARGUMENT);

  // OBTAIN CLIENT MEMBERS =====================================================
  auto client_data = static_cast<rmw_client_data_t *>(client->data);

  // CHECK SERVER AVAILABILITY =================================================
  // Check if server is alive by querying its availability Zenoh queryable
  zn_query(client_data->zn_session_,
           client->service_name,
           "",  // NOTE(CH3): Maybe use this predicate if we want to more things in the queryable
           zn_query_target_default(),
           zn_query_consolidation_default(),
           rmw_client_data_t::zn_service_availability_query_callback);

  std::string key(client->service_name);

  if (client_data->zn_availability_query_responses_.find(key) !=
    client_data->zn_availability_query_responses_.end())
  {
    client_data->zn_availability_query_responses_.erase(key);
    *result = true;
  } else {
    // TODO(CH3): Change this if a better way to throttle availability checks is found
    sleep(0.5); // Don't spam the service
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
  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[rmw_create_client] %s with queue of depth %ld",
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

  // CREATE CLIENT =============================================================
  rmw_client_t * client = static_cast<rmw_client_t *>(allocator->allocate(
    sizeof(rmw_client_t),
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

  client->data = static_cast<rmw_client_data_t *>(allocator->allocate(
    sizeof(rmw_client_data_t),
    allocator->state));
  new(client->data) rmw_client_data_t();
  if (!client->data) {
    RMW_SET_ERROR_MSG("failed to allocate client data");
    allocator->deallocate(const_cast<char *>(client->service_name), allocator->state);
    allocator->deallocate(client, allocator->state);
    return nullptr;
  }

  // CREATE CLIENT MEMBERS =====================================================
  // Get typed pointer to implementation specific client data struct
  auto client_data = static_cast<rmw_client_data_t *>(client->data);

  // Obtain Zenoh session and create Zenoh resource for request messages
  ZNSession * session = node->context->impl->session;
  client_data->zn_session_ = session;

  // Obtain qualified request-response topics
  std::string zn_topic_key(client->service_name);
  client_data->zn_request_topic_key_ = rcutils_strdup(
    (zn_topic_key + "/request").c_str(), *allocator);
  if (!client_data->zn_request_topic_key_) {
    RMW_SET_ERROR_MSG("failed to allocate zenoh request topic key");
    allocator->deallocate(client->data, allocator->state);

    allocator->deallocate(const_cast<char *>(client->service_name), allocator->state);
    allocator->deallocate(client, allocator->state);
    return nullptr;
  }

  client_data->zn_response_topic_key_ = rcutils_strdup(
    (zn_topic_key + "/response").c_str(), *allocator);
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
  client_data->zn_request_topic_id_ = zn_declare_resource(session, client_data->zn_request_topic_key_);

  // INSERT TYPE SUPPORT =======================================================
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
    allocator->deallocate(
      const_cast<char *>(client_data->zn_response_topic_key_), allocator->state);
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
    allocator->deallocate(const_cast<char *>(client_data->zn_response_topic_key_),
      allocator->state);
    allocator->deallocate(client_data->request_type_support_, allocator->state);
    allocator->deallocate(client->data, allocator->state);

    allocator->deallocate(const_cast<char *>(client->service_name), allocator->state);
    allocator->deallocate(client, allocator->state);
    return nullptr;
  }

  // CONFIGURE CLIENT ==========================================================
  // Assign node pointer
  client_data->node_ = node;

  // Assign and increment unique client ID atomically
  client_data->client_id_ =
    rmw_client_data_t::client_id_counter.fetch_add(1, std::memory_order_relaxed);

  // Configure response message queue
  client_data->queue_depth_ = qos_profile->depth;

  // ADD CLIENT DATA TO TOPIC MAP ==============================================
  // This will allow us to access the client data structs for this Zenoh topic key expression
  // (This is for listening for service responses)
  std::string topic_key(client_data->zn_response_topic_key_);
  auto topic_map_iter = rmw_client_data_t::zn_topic_to_client_data.find(topic_key);

  if (topic_map_iter == rmw_client_data_t::zn_topic_to_client_data.end()) {
    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[rmw_create_client] New response topic detected: %s",
      client_data->zn_response_topic_key_);

    // If no elements for this Zenoh topic key expression exists, add it in
    std::vector<rmw_client_data_t *> client_data_vec{client_data};
    rmw_client_data_t::zn_topic_to_client_data[topic_key] = client_data_vec;

    // We initialise subscribers ONCE (otherwise we'll get duplicate messages)
    // The topic name will be the same for any duplicate subscribers, so it is ok
    client_data->zn_response_subscriber_ = zn_declare_subscriber(
      client_data->zn_session_,
      client_data->zn_response_topic_key_,
      zn_subinfo_default(),  // NOTE(CH3): Default for now
      client_data->zn_response_sub_callback);

    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[rmw_create_client] Zenoh subscriber declared for %s",
      client_data->zn_response_topic_key_);
  } else {
    // Otherwise, append to the vector
    topic_map_iter->second.push_back(client_data);
  }

  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[rmw_create_client] Client for %s (ID: %ld) added to topic map",
    client_data->zn_response_topic_key_,
    client_data->client_id_);

  // ADD CLIENT DATA TO QUERYABLE MAP===========================================
  // This will allow us to access the client data structs for this Zenoh queryable key expression
  // (This is for checking service availability)
  std::string queryable_key(client->service_name);
  auto queryable_map_iter = rmw_client_data_t::zn_queryable_to_client_data.find(queryable_key);

  if (queryable_map_iter == rmw_client_data_t::zn_queryable_to_client_data.end()) {
    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[rmw_create_client] New queryable detected: %s",
      client->service_name);

    // If no elements for this Zenoh topic key expression exists, add it in
    std::vector<rmw_client_data_t *> client_data_vec_{client_data};
    rmw_client_data_t::zn_queryable_to_client_data[queryable_key] = client_data_vec_;
  } else {
    // Otherwise, append to the vector
    queryable_map_iter->second.push_back(client_data);
  }

  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[rmw_create_client] Client for %s (ID: %ld) added to queryable map",
    client->service_name,
    client_data->client_id_);

  // Spoof Zenoh queryable for availability checking
  //
  // NOTE(CH3): I know this code chunk looks super super random...
  // But it is necessary to resolve some as of yet unknown bug that causes the queryable on the
  // service server side to ignore the queries from a client (if the client is started first)
  //
  // I've tested this using pure Zenoh, and the bug doesn't arise. But somehow, despite using
  // exactly the same code in pure Zenoh, it doesn't want to work here in rmw_zenoh unless this
  // block is included.
  //
  // (Having a listener for queries on a separate process from the service seems to be all that is
  // necessary.
  //
  // Also, the issue is most likely happening on the SERVICE SERVER'session SIDE! But somehow adding this
  // queryable listener on the SERVICE CLIENT side fixes that issue.
  //
  // Additional note: Note that this means that for most use-cases (but not all), we should be fine.
  // the one edge case is if someone starts a service and client on the same process, but there is
  // a delay between when the client and service starts (and the client is started first, and there
  // are no other processes anywhere on the network where the Zenoh queryable is being listened to.)
  zn_declare_queryable(
    session,
    client->service_name,
    STORAGE,
    [](ZNQuery * query){});

  return client;
}

/// DESTROY SERVICE CLIENT =====================================================
// Destroy and deallocate an RMW service client
rmw_ret_t
rmw_destroy_client(rmw_node_t * node, rmw_client_t * client)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "[rmw_destroy_client] %s", client->service_name);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // OBTAIN CLIENT MEMBERS =====================================================
  auto client_data = static_cast<rmw_client_data_t *>(client->data);

  // DELETE CLIENT DATA IN TOPIC MAP ===========================================
  std::string key(client_data->zn_response_topic_key_);
  auto map_iter = rmw_client_data_t::zn_topic_to_client_data.find(key);

  if (map_iter == rmw_client_data_t::zn_topic_to_client_data.end()) {
    RCUTILS_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "client not found in Zenoh topic to client data map! %s",
      client_data->zn_response_topic_key_);
  } else {
    // Delete the subscription data pointer in the Zenoh topic to subscription data map
    for (auto it = map_iter->second.begin(); it != map_iter->second.end(); ++it) {
      if ((*it)->client_id_ == client_data->client_id_) {
        map_iter->second.erase(it);
        break;
      }
    }

    // Delete the map element if no other client data pointers exist
    // (That is, when no other clients are listening to the Zenoh response topic)
    if (map_iter->second.empty()) {
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp",
        "[rmw_destroy_client] No more clients listening to %s",
        client_data->zn_response_topic_key_);

      // Only when there are no more active RMW clients listening to this Zenoh topic, do we
      // undeclare the subscriber on Zenoh's end (which means no more Zenoh callbacks will trigger
      // on this topic)
      zn_undeclare_subscriber(client_data->zn_response_subscriber_);
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp",
        "[rmw_destroy_client] Zenoh subcriber undeclared for %s",
        client_data->zn_response_topic_key_);

      rmw_client_data_t::zn_topic_to_client_data.erase(map_iter);
    }

    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "[rmw_destroy_client] Client for %s (ID: %ld) removed from topic map",
      client_data->zn_response_topic_key_,
      client_data->client_id_);
  }

  // CLEANUP ===================================================================
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
  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[rmw_send_request] %s (%ld)",
    static_cast<rmw_client_data_t *>(client->data)->zn_request_topic_key_,
    static_cast<rmw_client_data_t *>(client->data)->zn_request_topic_id_);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(sequence_id, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  RMW_CHECK_FOR_NULL_WITH_MSG(
    client->data, "client implementation pointer is null", RMW_RET_INVALID_ARGUMENT);

  // OBTAIN CLIENT MEMBERS ====================================================
  auto * client_data = static_cast<rmw_client_data_t *>(client->data);

  // ASSIGN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator =
    &(static_cast<rmw_client_data_t *>(client->data)->node_->context->options.allocator);

  // SERIALIZE DATA ============================================================
  size_t max_data_length = (
    static_cast<rmw_client_data_t *>(client->data)
    ->request_type_support_->getEstimatedSerializedSize(ros_request));

  // Account for metadata
  max_data_length += sizeof(std::int64_t); // Internal type of the atomic sequence ID

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
    fastbuffer,
    eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
    eprosima::fastcdr::Cdr::DDS_CDR);

  if (!client_data->request_type_support_->serializeROSmessage(
      ros_request,
      ser,
      client_data->request_type_support_impl_)) {
    RMW_SET_ERROR_MSG("failed serialize ROS request message");
    allocator->deallocate(request_bytes, allocator->state);
    return RMW_RET_ERROR;
  }

  size_t data_length = ser.getSerializedDataLength();

  // ADD METADATA ==============================================================
  //
  // TODO(CH3): Refactor this into its own modular set of functions eventually to make adding
  // more metadata convenient
  *sequence_id = rmw_client_data_t::sequence_id_counter.fetch_add(1, std::memory_order_relaxed);

  size_t meta_length = sizeof(std::int64_t); // Internal type of the atomic sequence ID
  memcpy(request_bytes + data_length,
         reinterpret_cast<char *>(sequence_id),
         meta_length);

  // PUBLISH ON ZENOH MIDDLEWARE LAYER =========================================
  size_t wrid_ret = zn_write_wrid(
    client_data->zn_session_,
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
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_take_response");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
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
  std::unique_lock<std::mutex> lock(client_data->response_queue_mutex_);

  if (client_data->zn_response_message_queue_.empty()) {
    // NOTE(CH3): It is correct to be returning RMW_RET_OK. The information that the message
    // was not found is encoded in the fact that the taken-out parameter is still False.
    //
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }

  // NOTE(CH3): Potential place to handle "QoS" (e.g. could pop from back so it is LIFO)
  auto response_bytes_ptr = client_data->zn_response_message_queue_.back();
  client_data->zn_response_message_queue_.pop_back();

  client_data->response_queue_mutex_.unlock();

  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "[rmw_take] Response found: %s",
    client_data->zn_response_topic_key_);

  // RETRIEVE METADATA =========================================================
  // TODO(CH3): Again, refactor this into a modular set of functions eventually
  size_t meta_length = sizeof(std::int64_t); // Internal type of the atomic sequence ID

  // Use metadata
  memcpy(
    &request_header->request_id.sequence_number,
    &response_bytes_ptr->back() + 1 - meta_length,
    meta_length);

  // DESERIALIZE MESSAGE =======================================================
  size_t data_length = response_bytes_ptr->size() - meta_length;

  unsigned char * cdr_buffer = static_cast<unsigned char *>(
    allocator->allocate(data_length, allocator->state));
  memcpy(cdr_buffer, &response_bytes_ptr->front(), data_length);

  // Object that manages the raw buffer.
  eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char *>(cdr_buffer), data_length);

  // Object that deserializes the data
  eprosima::fastcdr::Cdr deser(
    fastbuffer,
    eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
    eprosima::fastcdr::Cdr::DDS_CDR);
  if (!client_data->response_type_support_->deserializeROSmessage(
      deser, ros_response, client_data->response_type_support_impl_))
    {
    RMW_SET_ERROR_MSG("could not deserialize ROS response message");
    return RMW_RET_ERROR;
  }

  *taken = true;
  allocator->deallocate(cdr_buffer, allocator->state);

  return RMW_RET_OK;
}
}  // extern "C"
