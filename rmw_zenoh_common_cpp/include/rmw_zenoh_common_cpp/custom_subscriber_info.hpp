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

#ifndef RMW_ZENOH_COMMON_CPP__CUSTOM_SUBSCRIBER_INFO_HPP_
#define RMW_ZENOH_COMMON_CPP__CUSTOM_SUBSCRIBER_INFO_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <set>
#include <utility>

#include "rmw_zenoh_common_cpp/zenoh-net-interface.h"

#include "rcpputils/thread_safety_annotations.hpp"

#include "rmw/impl/cpp/macros.hpp"

#include "rmw_zenoh_common_cpp/TypeSupport.hpp"


struct CustomSubscriberInfo
{
  virtual ~CustomSubscriberInfo() = default;

  zn_subscriber_t * subscriber_{nullptr};
  rmw_zenoh_common_cpp::TypeSupport * type_support_{nullptr};
  const void * type_support_impl_{nullptr};
  zn_reskey_t subscription_reskey_{};
  const char * typesupport_identifier_{nullptr};
};

#endif  // RMW_ZENOH_COMMON_CPP__CUSTOM_SUBSCRIBER_INFO_HPP_
