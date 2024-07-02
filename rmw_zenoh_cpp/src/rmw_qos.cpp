// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <cstdarg>

#include "rcutils/snprintf.h"

#include "rmw/error_handling.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

extern "C"
{
namespace
{
///=============================================================================
// Copied from rmw_dds_common::qos.cpp.
// Returns RMW_RET_OK if successful or no buffer was provided
// Returns RMW_RET_ERROR if there as an error copying the message to the buffer
rmw_ret_t
_append_to_buffer(char * buffer, size_t buffer_size, const char * format, ...)
{
  // Only write if a buffer is provided
  if (!buffer || buffer_size == 0u) {
    return RMW_RET_OK;
  }
  // Determine available space left in buffer
  size_t offset = strnlen(buffer, buffer_size);
  size_t write_size = buffer_size - offset;
  std::va_list args;
  va_start(args, format);
  int snprintf_ret = rcutils_vsnprintf(buffer + offset, write_size, format, args);
  va_end(args);
  if (snprintf_ret < 0) {
    RMW_SET_ERROR_MSG("failed to append to character buffer");
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}
}  // namespace

///=============================================================================
rmw_ret_t
rmw_qos_profile_check_compatible(
  const rmw_qos_profile_t publisher_profile,
  const rmw_qos_profile_t subscription_profile,
  rmw_qos_compatibility_type_t * compatibility,
  char * reason,
  size_t reason_size)
{
  if (!compatibility) {
    RMW_SET_ERROR_MSG("compatibility parameter is null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (!reason && reason_size != 0u) {
    RMW_SET_ERROR_MSG("reason parameter is null, but reason_size parameter is not zero");
    return RMW_RET_INVALID_ARGUMENT;
  }
  // Initialize reason buffer
  if (reason && reason_size != 0u) {
    reason[0] = '\0';
  }
  // In Zenoh, there are not qos incompatibilities.
  // Further publishers do not have any reliability settings.
  // The once scenario where transport may not occur is where a publisher with
  // TRANSIENT_LOCAL durability publishes a message before a subscription with
  // VOLATILE durability spins up. However, any subsequent messages published
  // will be received by the subscription.
  if (publisher_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL &&
    subscription_profile.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE)
  {
    *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
    return _append_to_buffer(
      reason,
      reason_size,
      "WARNING: Publisher's durability is TRANSIENT_LOCAL, but subscription's is VOLATILE;");
  } else {
    *compatibility = RMW_QOS_COMPATIBILITY_OK;
  }
  return RMW_RET_OK;
}
}  // extern "C"
