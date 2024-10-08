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

#include "rmw_context_impl_s.hpp"

#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "identifier.hpp"
#include "liveliness_utils.hpp"
#include "logging_macros.hpp"
#include "zenoh_config.hpp"
#include "zenoh_router_check.hpp"

#include "rcpputils/scope_exit.hpp"
#include "rmw/error_handling.h"

// Megabytes of SHM to reserve.
// TODO(clalancette): Make this configurable, or get it from the configuration
#define SHM_BUFFER_SIZE_MB 10

///=============================================================================
void rmw_context_impl_s::graph_sub_data_handler(z_loaned_sample_t * sample, void * data)
{
  z_view_string_t keystr;
  z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);

  auto data_ptr = static_cast<Data *>(data);
  if (data_ptr == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "[graph_sub_data_handler] Invalid data_ptr."
    );
    return;
  }

  // Update the graph cache.
  std::lock_guard<std::recursive_mutex> lock(data_ptr->mutex_);
  if (data_ptr->is_shutdown_) {
    return;
  }
  std::string str(z_string_data(z_loan(keystr)), z_string_len(z_loan(keystr)));
  switch (z_sample_kind(sample)) {
    case z_sample_kind_t::Z_SAMPLE_KIND_PUT:
      data_ptr->graph_cache_->parse_put(str);
      break;
    case z_sample_kind_t::Z_SAMPLE_KIND_DELETE:
      data_ptr->graph_cache_->parse_del(str);
      break;
    default:
      return;
  }

  // Trigger the ROS graph guard condition.
  rmw_ret_t rmw_ret = rmw_trigger_guard_condition(data_ptr->graph_guard_condition_.get());
  if (RMW_RET_OK != rmw_ret) {
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "[graph_sub_data_handler] Unable to trigger graph guard condition."
    );
  }
}

///=============================================================================
rmw_context_impl_s::Data::Data(
  std::size_t domain_id,
  const std::string & enclave,
  z_owned_session_t session,
  std::optional<z_owned_shm_provider_t> shm_provider,
  const std::string & liveliness_str,
  std::shared_ptr<rmw_zenoh_cpp::GraphCache> graph_cache)
: enclave_(std::move(enclave)),
  domain_id_(std::move(domain_id)),
  session_(std::move(session)),
  shm_provider_(std::move(shm_provider)),
  liveliness_str_(std::move(liveliness_str)),
  graph_cache_(std::move(graph_cache)),
  is_shutdown_(false),
  next_entity_id_(0),
  is_initialized_(false),
  nodes_({})
{
  graph_guard_condition_ = std::make_unique<rmw_guard_condition_t>();
  graph_guard_condition_->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  graph_guard_condition_->data = &guard_condition_data_;
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::Data::subscribe_to_ros_graph()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (is_initialized_) {
    return RMW_RET_OK;
  }
  // Setup the liveliness subscriber to receives updates from the ROS graph
  // and update the graph cache.
  // TODO(Yadunund): This closure is still not 100% thread safe as we are
  // passing Data* as the type erased argument to z_closure. Thus during
  // the execution of graph_sub_data_handler, the rawptr may be freed/reset
  // by a different thread. When we switch to zenoh-cpp we can replace z_closure
  // with a lambda that captures a weak_ptr<Data> by copy. The lambda and caputed
  // weak_ptr<Data> will have the same lifetime as the subscriber. Then within
  // graph_sub_data_handler, we would first lock to weak_ptr to check if the
  // shared_ptr<Data> exits. If it does, then even if a different thread calls
  // rmw_context_fini() to destroy rmw_context_impl_s, the locked
  // shared_ptr<Data> would live on until the graph_sub_data_handler callback.
  zc_liveliness_subscriber_options_t sub_options;
  zc_liveliness_subscriber_options_default(&sub_options);
  // Enable history option to retrieve the old graph info in case there is
  // something updated after getting the graph info.
  sub_options.history = true;
  z_owned_closure_sample_t callback;
  z_closure(&callback, graph_sub_data_handler, nullptr, this);
  z_view_keyexpr_t keyexpr;
  z_view_keyexpr_from_str(&keyexpr, liveliness_str_.c_str());
  auto undeclare_z_sub = rcpputils::make_scope_exit(
    [this]() {
      z_undeclare_subscriber(z_move(this->graph_subscriber_));
    });
  if (zc_liveliness_declare_subscriber(
      &graph_subscriber_,
      z_loan(session_), z_loan(keyexpr),
      z_move(callback), &sub_options) != Z_OK)
  {
    RMW_SET_ERROR_MSG("unable to create zenoh subscription");
    return RMW_RET_ERROR;
  }

  undeclare_z_sub.cancel();
  is_initialized_ = true;
  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::Data::shutdown()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  rmw_ret_t ret = RMW_RET_OK;
  if (is_shutdown_) {
    return ret;
  }

  // Shutdown all the nodes in this context.
  for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
    ret = node_it->second->shutdown();
    if (ret != RMW_RET_OK) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Unable to shutdown node with id %zu. rmw_ret_t code: %zu.",
        node_it->second->id(),
        ret
      );
    }
  }

  z_undeclare_subscriber(z_move(graph_subscriber_));
  if (shm_provider_.has_value()) {
    z_drop(z_move(shm_provider_.value()));
  }
  // Close the zenoh session
  if (z_close(z_move(session_), NULL) != Z_OK) {
    RMW_SET_ERROR_MSG("Error while closing zenoh session");
    return RMW_RET_ERROR;
  }
  is_shutdown_ = true;
  return RMW_RET_OK;
}

///=============================================================================
rmw_context_impl_s::Data::~Data()
{
  auto ret = this->shutdown();
  nodes_.clear();
  static_cast<void>(ret);
}

///=============================================================================
rmw_context_impl_s::rmw_context_impl_s(
  const std::size_t domain_id,
  const std::string & enclave)
{
  // Initialize the zenoh configuration.
  z_owned_config_t config;
  rmw_ret_t ret;
  if ((ret =
    rmw_zenoh_cpp::get_z_config(
      rmw_zenoh_cpp::ConfigurableEntity::Session,
      &config)) != RMW_RET_OK)
  {
    throw std::runtime_error("Error configuring Zenoh session.");
  }

  // Check if shm is enabled.
  z_owned_string_t shm_enabled;
  zc_config_get_from_str(z_loan(config), Z_CONFIG_SHARED_MEMORY_KEY, &shm_enabled);
  auto always_free_shm_enabled = rcpputils::make_scope_exit(
    [&shm_enabled]() {
      z_drop(z_move(shm_enabled));
    });

  // Initialize the zenoh session.
  z_owned_session_t session;
  if (z_open(&session, z_move(config), NULL) != Z_OK) {
    RMW_SET_ERROR_MSG("Error setting up zenoh session");
    throw std::runtime_error("Error setting up zenoh session.");
  }
  auto close_session = rcpputils::make_scope_exit(
    [&session]() {
      z_close(z_move(session), NULL);
    });

  // TODO(Yadunund) Move this check into a separate thread.
  // Verify if the zenoh router is running if configured.
  const std::optional<uint64_t> configured_connection_attempts =
    rmw_zenoh_cpp::zenoh_router_check_attempts();
  if (configured_connection_attempts.has_value()) {
    ret = RMW_RET_ERROR;
    uint64_t connection_attempts = 0;
    // Retry until the connection is successful.
    while (ret != RMW_RET_OK && connection_attempts < configured_connection_attempts.value()) {
      if ((ret = rmw_zenoh_cpp::zenoh_router_check(z_loan(session))) != RMW_RET_OK) {
        ++connection_attempts;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (ret != RMW_RET_OK) {
      throw std::runtime_error(
              "Unable to connect to a Zenoh router after " +
              std::to_string(configured_connection_attempts.value()) +
              " retries.");
    }
  }

  // Initialize the graph cache.
  const z_id_t zid = z_info_zid(z_loan(session));
  auto graph_cache = std::make_shared<rmw_zenoh_cpp::GraphCache>(zid);
  // Setup liveliness subscriptions for discovery.
  std::string liveliness_str = rmw_zenoh_cpp::liveliness::subscription_token(
    domain_id);

  // Query router/liveliness participants to get graph information before this session was started.
  // We create a blocking channel that is unbounded, ie. `bound` = 0, to receive
  // replies for the zc_liveliness_get() call. This is necessary as if the `bound`
  // is too low, the channel may starve the zenoh executor of its threads which
  // would lead to deadlocks when trying to receive replies and block the
  // execution here.
  // The blocking channel will return when the sender end is closed which is
  // the moment the query finishes.
  // The non-blocking fifo exists only for the use case where we don't want to
  // block the thread between responses (including the request termination response).
  // In general, unless we want to cooperatively schedule other tasks on the same
  // thread as reading the fifo, the blocking fifo will be more appropriate as
  // the code will be simpler, and if we're just going to spin over the non-blocking
  // reads until we obtain responses, we'll just be hogging CPU time by convincing
  // the OS that we're doing actual work when it could instead park the thread.
  z_owned_fifo_handler_reply_t handler;
  z_owned_closure_reply_t closure;
  z_fifo_channel_reply_new(&closure, &handler, SIZE_MAX - 1);

  z_view_keyexpr_t keyexpr;
  z_view_keyexpr_from_str(&keyexpr, liveliness_str.c_str());
  zc_liveliness_get(
    z_loan(session), z_loan(keyexpr),
    z_move(closure), NULL);

  z_owned_reply_t reply;
  while (z_recv(z_loan(handler), &reply) == Z_OK) {
    if (z_reply_is_ok(z_loan(reply))) {
      const z_loaned_sample_t * sample = z_reply_ok(z_loan(reply));
      z_view_string_t key_str;
      z_keyexpr_as_view_string(z_sample_keyexpr(sample), &key_str);
      std::string str(z_string_data(z_loan(key_str)), z_string_len(z_loan(key_str)));
      // Ignore tokens from the same session to avoid race conditions from this
      // query and the liveliness subscription.
      graph_cache->parse_put(str, true);
    } else {
      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "[rmw_context_impl_s] z_call received an invalid reply.\n");
    }
    z_drop(z_move(reply));
  }
  z_drop(z_move(handler));

  // Initialize the shm manager if shared_memory is enabled in the config.
  std::optional<z_owned_shm_provider_t> shm_provider = std::nullopt;
  if (strncmp(z_string_data(z_loan(shm_enabled)), "true", z_string_len(z_loan(shm_enabled))) == 0) {
    RMW_ZENOH_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "SHM is enabled");

    // TODO(yuyuan): determine the default alignment of SHM
    z_alloc_alignment_t alignment = {5};
    z_owned_memory_layout_t layout;
    z_memory_layout_new(&layout, SHM_BUFFER_SIZE_MB * 1024 * 1024, alignment);

    z_owned_shm_provider_t provider;
    if (z_posix_shm_provider_new(&provider, z_loan(layout)) != Z_OK) {
      RMW_ZENOH_LOG_ERROR_NAMED("rmw_zenoh_cpp", "Unable to create a SHM provider.");
      throw std::runtime_error("Unable to create shm manager.");
    }
    shm_provider = provider;
  }
  auto free_shm_provider = rcpputils::make_scope_exit(
    [&shm_provider]() {
      if (shm_provider.has_value()) {
        z_drop(z_move(shm_provider.value()));
      }
    });

  close_session.cancel();
  free_shm_provider.cancel();

  data_ = std::make_shared<Data>(
    domain_id,
    std::move(enclave),
    std::move(session),
    std::move(shm_provider),
    std::move(liveliness_str),
    std::move(graph_cache));

  ret = data_->subscribe_to_ros_graph();
  if (ret != RMW_RET_OK) {
    throw std::runtime_error("Unable to subscribe to ROS Graph updates.");
  }
}

///=============================================================================
std::string rmw_context_impl_s::enclave() const
{
  std::lock_guard<std::recursive_mutex> lock(data_->mutex_);
  return data_->enclave_;
}

///=============================================================================
const z_loaned_session_t * rmw_context_impl_s::session() const
{
  std::lock_guard<std::recursive_mutex> lock(data_->mutex_);
  return z_loan(data_->session_);
}

///=============================================================================
std::optional<z_owned_shm_provider_t> & rmw_context_impl_s::shm_provider()
{
  std::lock_guard<std::recursive_mutex> lock(data_->mutex_);
  return data_->shm_provider_;
}

///=============================================================================
rmw_guard_condition_t * rmw_context_impl_s::graph_guard_condition()
{
  std::lock_guard<std::recursive_mutex> lock(data_->mutex_);
  return data_->graph_guard_condition_.get();
}

///=============================================================================
std::size_t rmw_context_impl_s::get_next_entity_id()
{
  std::lock_guard<std::recursive_mutex> lock(data_->mutex_);
  return data_->next_entity_id_++;
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::shutdown()
{
  return data_->shutdown();
}

///=============================================================================
bool rmw_context_impl_s::is_shutdown() const
{
  std::lock_guard<std::recursive_mutex> lock(data_->mutex_);
  return data_->is_shutdown_;
}

///=============================================================================
std::shared_ptr<rmw_zenoh_cpp::GraphCache> rmw_context_impl_s::graph_cache()
{
  std::lock_guard<std::recursive_mutex> lock(data_->mutex_);
  return data_->graph_cache_;
}

///=============================================================================
bool rmw_context_impl_s::create_node_data(
  const rmw_node_t * const node,
  const std::string & ns,
  const std::string & node_name)
{
  std::lock_guard<std::recursive_mutex> lock(data_->mutex_);
  if (data_->nodes_.count(node) > 0) {
    // Node already exists.
    return false;
  }

  auto node_data = rmw_zenoh_cpp::NodeData::make(
    node,
    this->get_next_entity_id(),
    z_loan(data_->session_),
    data_->domain_id_,
    ns,
    node_name,
    data_->enclave_);
  if (node_data == nullptr) {
    // Error already handled.
    return false;
  }

  auto node_insertion = data_->nodes_.insert(std::make_pair(node, std::move(node_data)));
  if (!node_insertion.second) {
    return false;
  }

  return true;
}

///=============================================================================
std::shared_ptr<rmw_zenoh_cpp::NodeData> rmw_context_impl_s::get_node_data(
  const rmw_node_t * const node)
{
  std::lock_guard<std::recursive_mutex> lock(data_->mutex_);
  auto node_it = data_->nodes_.find(node);
  if (node_it == data_->nodes_.end()) {
    return nullptr;
  }
  return node_it->second;
}

///=============================================================================
void rmw_context_impl_s::delete_node_data(const rmw_node_t * const node)
{
  std::lock_guard<std::recursive_mutex> lock(data_->mutex_);
  data_->nodes_.erase(node);
}
