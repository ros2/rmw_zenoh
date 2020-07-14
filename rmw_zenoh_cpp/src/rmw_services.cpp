#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

extern "C"
{

rmw_client_t *
rmw_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  (void)node;
  (void)type_support;
  (void)service_name;
  (void)qos_profile;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_create_client");
  return nullptr;
}

rmw_ret_t
rmw_destroy_client(rmw_node_t * node, rmw_client_t * client)
{
  (void)node;
  (void)client;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_destroy_client");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_send_request(const rmw_client_t * client, const void * ros_request, int64_t * sequence_id)
{
  (void)client;
  (void)ros_request;
  (void)sequence_id;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_send_request");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_take_response(
  const rmw_client_t * client,
  rmw_service_info_t * request_header,
  void * ros_response,
  bool * taken)
{
  (void)client;
  (void)request_header;
  (void)ros_response;
  (void)taken;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take_response");
  return RMW_RET_ERROR;
}

rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  (void)node;
  (void)type_support;
  (void)service_name;
  (void)qos_profile;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_create_service");
  return nullptr;
}

rmw_ret_t
rmw_destroy_service(rmw_node_t * node, rmw_service_t * service)
{
  (void)node;
  (void)service;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_destroy_service");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_take_request(
  const rmw_service_t * service,
  rmw_service_info_t * request_header,
  void * ros_request,
  bool * taken)
{
  (void)service;
  (void)request_header;
  (void)ros_request;
  (void)taken;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take_request");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header,
  void * ros_response)
{
  (void)service;
  (void)request_header;
  (void)ros_response;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_send_response");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * result)
{
  (void)node;
  (void)client;
  (void)result;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_service_server_is_available");
  return RMW_RET_ERROR;
}

} // extern "C"
