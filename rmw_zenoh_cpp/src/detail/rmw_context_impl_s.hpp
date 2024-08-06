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
  using GraphCacheEventCallback = rmw_zenoh_cpp::GraphCache::GraphCacheEventCallback;

  // Constructor.
  // Once constructed, the context_impl instanced will manage the lifetime
  // of these arguments.
  rmw_context_impl_s(
    const rcutils_allocator_t * allocator,
    const std::size_t domain_id,
    const std::string & enclave,
    z_owned_session_t session,
    std::optional<zc_owned_shm_manager_t> shm_manager,
    rmw_guard_condition_t * graph_guard_condition);

  // Get a copy of the enclave.
  std::string enclave() const;

  // Loan the Zenoh session.
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

  rmw_ret_t get_node_names(
    rcutils_string_array_t * node_names,
    rcutils_string_array_t * node_namespaces,
    rcutils_string_array_t * enclaves,
    rcutils_allocator_t * allocator) const;

  rmw_ret_t get_topic_names_and_types(
    rcutils_allocator_t * allocator,
    bool no_demangle,
    rmw_names_and_types_t * topic_names_and_types) const;

  rmw_ret_t publisher_count_matched_subscriptions(
    const rmw_publisher_t * publisher,
    size_t * subscription_count);

  rmw_ret_t subscription_count_matched_publishers(
    const rmw_subscription_t * subscription,
    size_t * publisher_count);

  rmw_ret_t get_service_names_and_types(
    rcutils_allocator_t * allocator,
    rmw_names_and_types_t * service_names_and_types) const;

  rmw_ret_t count_publishers(
    const char * topic_name,
    size_t * count) const;

  rmw_ret_t count_subscriptions(
    const char * topic_name,
    size_t * count) const;

  rmw_ret_t count_services(
    const char * service_name,
    size_t * count) const;

  rmw_ret_t count_clients(
    const char * service_name,
    size_t * count) const;

  rmw_ret_t get_entity_names_and_types_by_node(
    rmw_zenoh_cpp::liveliness::EntityType entity_type,
    rcutils_allocator_t * allocator,
    const char * node_name,
    const char * node_namespace,
    bool no_demangle,
    rmw_names_and_types_t * names_and_types) const;

  rmw_ret_t get_entities_info_by_topic(
    rmw_zenoh_cpp::liveliness::EntityType entity_type,
    rcutils_allocator_t * allocator,
    const char * topic_name,
    bool no_demangle,
    rmw_topic_endpoint_info_array_t * endpoints_info) const;

  rmw_ret_t service_server_is_available(
    const char * service_name,
    const char * service_type,
    bool * is_available) const;

  void set_qos_event_callback(
    rmw_zenoh_cpp::liveliness::ConstEntityPtr entity,
    const rmw_zenoh_cpp::rmw_zenoh_event_type_t & event_type,
    GraphCacheEventCallback callback);

private:
  // Bundle all class members into a data struct which can be passed as a
  // weak ptr to various threads for thread-safe access without capturing
  // "this" ptr by reference.
  struct Data : public std::enable_shared_from_this<Data>
  {
    // Constructor.
    Data(
      const rcutils_allocator_t * allocator,
      const std::size_t domain_id,
      const std::string & enclave,
      z_owned_session_t session,
      std::optional<zc_owned_shm_manager_t> shm_manager,
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
    // ROS graph liveliness subscriber.
    z_owned_subscriber_t graph_subscriber_;
    // Equivalent to rmw_dds_common::Context's guard condition
    /// Guard condition that should be triggered when the graph changes.
    rmw_guard_condition_t * graph_guard_condition_;
    /// Shutdown flag.
    bool is_shutdown_;
    // A counter to assign a local id for every entity created in this session.
    size_t next_entity_id_;
    // Graph cache.
    std::unique_ptr<rmw_zenoh_cpp::GraphCache> graph_cache_;
    // True once graph subscriber is initialized.
    bool is_initialized_;
  };

  std::shared_ptr<Data> data_{nullptr};

  static void graph_sub_data_handler(const z_sample_t * sample, void * data);
};


#endif  // DETAIL__RMW_CONTEXT_IMPL_S_HPP_
