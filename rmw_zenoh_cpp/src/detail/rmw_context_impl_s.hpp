// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#ifndef DETAIL__RMW_CONTEXT_IMPL_S_HPP_
#define DETAIL__RMW_CONTEXT_IMPL_S_HPP_

#include <zenoh.h>

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "graph_cache.hpp"
#include "liveliness_utils.hpp"

#include "rcutils/types.h"
#include "rmw/rmw.h"

///=============================================================================
class rmw_context_impl_s final
{
public:
  // Constructor that internally initializees the Zenoh session and other artifacts.
  // Throws an std::runtime_error if any of the initializations fail.
  // The construction will block until a Zenoh router is detected.
  // TODO(Yadunund): Make this a non-blocking call by checking for the Zenoh
  // router in a separate thread. Instead block when creating a node if router
  // check has not succeeded.
  rmw_context_impl_s(
    const rcutils_allocator_t * allocator,
    const std::size_t domain_id,
    const std::string & enclave);

  // Get a copy of the enclave.
  std::string enclave() const;

  // Loan the Zenoh session.
  // TODO(Yadunund): Remove this API once rmw_context_impl_s is updated to
  // create other Zenoh objects.
  z_session_t session() const;

  // Get a reference to the shm_manager.
  // Note: This is not thread-safe.
  // TODO(Yadunund): Remove this API and instead include a publish() API
  // that handles the shm_manager once the context manages publishers.
  std::optional<zc_owned_shm_manager_t> & shm_manager();

  // Get the graph guard condition.
  rmw_guard_condition_t * graph_guard_condition();

  // Get a unique id for a new entity.
  size_t get_next_entity_id();

  // Shutdown the Zenoh session.
  rmw_ret_t shutdown();

  // Check if the Zenoh session is shutdown.
  bool is_shutdown() const;

  // Returns true if the Zenoh session is valid.
  bool session_is_valid() const;

  /// Return a shared_ptr to the GraphCache stored in this context.
  std::shared_ptr<rmw_zenoh_cpp::GraphCache> graph_cache();

private:
  // Bundle all class members into a data struct which can be passed as a
  // weak ptr to various threads for thread-safe access without capturing
  // "this" ptr by reference.
  struct Data : public std::enable_shared_from_this<Data>
  {
    // Constructor.
    Data(
      const rcutils_allocator_t * allocator,
      const std::string & enclave,
      z_owned_session_t session,
      std::optional<zc_owned_shm_manager_t> shm_manager,
      const std::string & liveliness_str,
      std::shared_ptr<rmw_zenoh_cpp::GraphCache> graph_cache,
      rmw_guard_condition_t * graph_guard_condition);

    // Subscribe to the ROS graph.
    rmw_ret_t subscribe();

    // Shutdown the Zenoh session.
    rmw_ret_t shutdown();

    // Destructor.
    ~Data();

    // Mutex to lock when accessing members.
    mutable std::mutex mutex_;
    // RMW allocator.
    const rcutils_allocator_t * allocator_;
    // Enclave, name used to find security artifacts in a sros2 keystore.
    std::string enclave_;
    // An owned session.
    z_owned_session_t session_;
    // An optional SHM manager that is initialized of SHM is enabled in the
    // zenoh session config.
    std::optional<zc_owned_shm_manager_t> shm_manager_;
    // Liveliness keyexpr string to subscribe to for ROS graph changes.
    std::string liveliness_str_;
    // Graph cache.
    std::shared_ptr<rmw_zenoh_cpp::GraphCache> graph_cache_;
    // ROS graph liveliness subscriber.
    z_owned_subscriber_t graph_subscriber_;
    // Equivalent to rmw_dds_common::Context's guard condition
    /// Guard condition that should be triggered when the graph changes.
    rmw_guard_condition_t * graph_guard_condition_;
    /// Shutdown flag.
    bool is_shutdown_;
    // A counter to assign a local id for every entity created in this session.
    size_t next_entity_id_;
    // True once graph subscriber is initialized.
    bool is_initialized_;
  };

  std::shared_ptr<Data> data_{nullptr};

  static void graph_sub_data_handler(const z_sample_t * sample, void * data);
};


#endif  // DETAIL__RMW_CONTEXT_IMPL_S_HPP_
