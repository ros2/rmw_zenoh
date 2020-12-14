// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef RMW_ZENOH_COMMON_CPP__CUSTOM_SESSION_INFO_HPP_
#define RMW_ZENOH_COMMON_CPP__CUSTOM_SESSION_INFO_HPP_

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "rcpputils/thread_safety_annotations.hpp"
#include "rcutils/logging_macros.h"

#include "rmw/impl/cpp/key_value.hpp"
#include "rmw/qos_profiles.h"
#include "rmw/rmw.h"

#include "rmw_zenoh_common_cpp/zenoh-net-interface.h"

typedef struct CustomSessionInfo
{
  zn_session_t * session;
} CustomSessionInfo;

#endif  // RMW_ZENOH_COMMON_CPP__CUSTOM_SESSION_INFO_HPP_
