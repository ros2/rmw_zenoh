// Copyright 2020 ADLINK, Inc.
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

#ifndef RMW_ZENOH_DYNAMIC_CPP__PUBLISHER_HPP_
#define RMW_ZENOH_DYNAMIC_CPP__PUBLISHER_HPP_

#include "rmw/rmw.h"
#include "rmw_zenoh_common_cpp/custom_session_info.hpp"

namespace rmw_zenoh_dynamic_cpp
{

rmw_publisher_t *
create_publisher(
  const CustomSessionInfo * session_info,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies,
  const rmw_publisher_options_t * publisher_options);

}  // namespace rmw_zenoh_dynamic_cpp

#endif  // RMW_ZENOH_DYNAMIC_CPP__PUBLISHER_HPP_
