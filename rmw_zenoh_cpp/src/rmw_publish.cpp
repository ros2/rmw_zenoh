#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

#include "rcutils/logging_macros.h"

#include <rmw/validate_full_topic_name.h>
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/rmw.h"

#include "rmw_zenoh_cpp/zenoh_pubsub.hpp"
#include "rmw_zenoh_cpp/identifier.hpp"

#include "type_support_common.hpp"

extern "C"
{
#include "zenoh/zenoh-ffi.h"

/// PUBLISH ROS MESSAGE ========================================================
// Serialize and publish a ROS message using Zenoh.
rmw_ret_t
rmw_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void) allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp",
                         "rmw_publish to %s, %ld",
                         publisher->topic_name,
                         static_cast<rmw_publisher_data_t *>(publisher->data)->zn_topic_id_);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION
  );

  auto publisher_data = static_cast<rmw_publisher_data_t *>(publisher->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher_data, "publisher data pointer is null", return RMW_RET_ERROR
  );

  // TODO(CH3): Properly implement the allocation
  rmw_zenoh_cpp::SerializedData * ser_data = new rmw_zenoh_cpp::SerializedData;

  // Message specific serialisation callback functions
  ser_data->impl = publisher_data->type_support_impl_;

  // size_t data_length = strlen(static_cast<const char *>(ros_message)) * sizeof(ros_message);
  size_t data_length = (
    static_cast<rmw_publisher_data_t *>(publisher->data)
    ->type_support_
    ->getEstimatedSerializedSize(ros_message)
  );

  ser_data->data = new char[data_length];
  ser_data->max_size = data_length;

  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(ser_data->data),
    data_length
  );  // Object that manages the raw buffer.

  eprosima::fastcdr::Cdr ser(fastbuffer,
                             eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                             eprosima::fastcdr::Cdr::DDS_CDR);  // Object that serializes the data.
  if (publisher_data->type_support_->serializeROSmessage(const_cast<void *>(ros_message),
                                                         ser,
                                                         ser_data->impl)) {
    // payload->encapsulation = ser.endianness() ==
      // eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    ser_data->length = (uint32_t)ser.getSerializedDataLength();
  } else {
    return RMW_RET_ERROR;
  }

  // PUBLISH ON ZENOH MIDDLEWARE LAYER =========================================
  zn_write_wrid(publisher_data->zn_session_,
                publisher_data->zn_topic_id_,
                // static_cast<char *>(ser_data->data),
                reinterpret_cast<char *>(ser_data),  // Convert into raw bytes
                ser_data->length);

  RCUTILS_LOG_INFO("DATA OF SIZE %ld: %s",
                   sizeof(ser_data),
                   reinterpret_cast<char *>(ser_data));

  delete static_cast<char *>(ser_data->data);
  delete ser_data;

  // if (!info->publisher_->write(&data)) {
  //   RMW_SET_ERROR_MSG("cannot publish data");
  //   return RMW_RET_ERROR;
  // }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)serialized_message;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publish_serialized_message");
  return RMW_RET_ERROR;
}

rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)publisher;
  (void)ros_message;
  (void)allocation;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_publish_loaned_message");
  return RMW_RET_ERROR;
}

} // extern "C"
