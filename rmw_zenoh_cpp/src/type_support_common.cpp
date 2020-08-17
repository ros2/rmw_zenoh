#include <string>

#include "rmw/error_handling.h"

#include "type_support_common.hpp"

namespace rmw_zenoh_cpp
{

TypeSupport::TypeSupport()
{
  max_size_bound_ = false;
}

void TypeSupport::set_members(const message_type_support_callbacks_t * members)
{
  members_ = members;

  // Fully bound by default
  max_size_bound_ = true;
  auto data_size = static_cast<uint32_t>(members->max_serialized_size(max_size_bound_));

  // A fully bound message of size 0 is an empty message
  if (max_size_bound_ && (data_size == 0) ) {
    has_data_ = false;
    ++data_size;  // Dummy byte
  } else {
    has_data_ = true;
  }

  // Total size is encapsulation size + data size
  type_size_ = 4 + data_size;
}

size_t TypeSupport::getEstimatedSerializedSize(const void * ros_message)
{
  if (max_size_bound_) {
    return type_size_;
  }

  assert(ros_message);

  // Encapsulation size + message size
  return 4 + members_->get_serialized_size(ros_message);
}

bool TypeSupport::serializeROSmessage(const void * ros_message,
                                      eprosima::fastcdr::Cdr & ser,
                                      const void * impl) const
{
  assert(ros_message);
  assert(impl);

  // Serialize encapsulation
  ser.serialize_encapsulation();

  // If type is not empty, serialize message
  if (has_data_) {
    auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);
    return callbacks->cdr_serialize(ros_message, ser);
  }

  // Otherwise, add a dummy byte
  ser << (uint8_t)0;
  return true;
}

bool TypeSupport::deserializeROSmessage(eprosima::fastcdr::Cdr & deser,
                                        void * ros_message,
                                        const void * impl) const
{
  assert(ros_message);
  assert(impl);

  // Deserialize encapsulation.
  deser.read_encapsulation();

  // If type is not empty, deserialize message
  if (has_data_) {
    auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);
    return callbacks->cdr_deserialize(deser, ros_message);
  }

  // Otherwise, consume dummy byte
  uint8_t dump = 0;
  deser >> dump;
  (void)dump;

  return true;
}

MessageTypeSupport::MessageTypeSupport(const message_type_support_callbacks_t * members)
{
  assert(members);

  // std::string name = _create_type_name(members);
  // this->setName(name.c_str());

  set_members(members);
}



// ServiceTypeSupport::ServiceTypeSupport()
// {
// }
//
// RequestTypeSupport::RequestTypeSupport(const service_type_support_callbacks_t * members)
// {
//   assert(members);
//
//   auto msg = static_cast<const message_type_support_callbacks_t *>(
//     members->request_members_->data);
//   std::string name = _create_type_name(msg);  // + "Request_";
//   this->setName(name.c_str());
//
//   set_members(msg);
// }
//
// ResponseTypeSupport::ResponseTypeSupport(const service_type_support_callbacks_t * members)
// {
//   assert(members);
//
//   auto msg = static_cast<const message_type_support_callbacks_t *>(
//     members->response_members_->data);
//   std::string name = _create_type_name(msg);  // + "Response_";
//   this->setName(name.c_str());
//
//   set_members(msg);
// }

}  // namespace rmw_zenoh_cpp
