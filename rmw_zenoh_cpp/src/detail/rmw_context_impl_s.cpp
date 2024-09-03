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

#include <utility>
#include <thread>

#include "guard_condition.hpp"
#include "identifier.hpp"
#include "liveliness_utils.hpp"
#include "logging_macros.hpp"
#include "zenoh_config.hpp"
#include "zenoh_router_check.hpp"

#include "rcpputils/scope_exit.hpp"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"

// Megabytes of SHM to reserve.
// TODO(clalancette): Make this configurable, or get it from the configuration
#define SHM_BUFFER_SIZE_MB 10

///=============================================================================
void rmw_context_impl_s::graph_sub_data_handler(const z_sample_t * sample, void * data)
{
  z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
  auto free_keystr = rcpputils::make_scope_exit(
    [&keystr]() {
      z_drop(z_move(keystr));
    });

  auto data_ptr = static_cast<Data *>(data);
  if (data_ptr == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "[graph_sub_data_handler] Invalid data_ptr."
    );
    return;
  }

  // Update the graph cache.
  std::lock_guard<std::mutex> lock(data_ptr->mutex_);
  if (data_ptr->is_shutdown_) {
    return;
  }
  switch (sample->kind) {
    case z_sample_kind_t::Z_SAMPLE_KIND_PUT:
      data_ptr->graph_cache_->parse_put(keystr._cstr);
      break;
    case z_sample_kind_t::Z_SAMPLE_KIND_DELETE:
      data_ptr->graph_cache_->parse_del(keystr._cstr);
      break;
    default:
      return;
  }

  // Trigger the ROS graph guard condition.
  rmw_ret_t rmw_ret = rmw_trigger_guard_condition(data_ptr->graph_guard_condition_);
  if (RMW_RET_OK != rmw_ret) {
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "[graph_sub_data_handler] Unable to trigger graph guard condition."
    );
  }
}

///=============================================================================
rmw_context_impl_s::Data::Data(
  const rcutils_allocator_t * allocator,
  const std::string & enclave,
  z_owned_session_t session,
  std::optional<zc_owned_shm_manager_t> shm_manager,
  const std::string & liveliness_str,
  std::shared_ptr<rmw_zenoh_cpp::GraphCache> graph_cache,
  rmw_guard_condition_t * graph_guard_condition)
: allocator_(allocator),
  enclave_(std::move(enclave)),
  session_(std::move(session)),
  shm_manager_(std::move(shm_manager)),
  liveliness_str_(std::move(liveliness_str)),
  graph_cache_(std::move(graph_cache)),
  graph_guard_condition_(graph_guard_condition),
  is_shutdown_(false),
  next_entity_id_(0),
  is_initialized_(false)
{
  // Do nothing.
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::Data::subscribe()
{
  if (is_initialized_) {
    return RMW_RET_OK;
  }
  // Setup the liveliness subscriber to receives updates from the ROS graph
  // and update the graph cache.
  auto sub_options = zc_liveliness_subscriber_options_null();
  z_owned_closure_sample_t callback = z_closure(
    rmw_context_impl_s::graph_sub_data_handler, nullptr,
    this->shared_from_this().get());
  graph_subscriber_ = zc_liveliness_declare_subscriber(
    z_loan(session_),
    z_keyexpr(liveliness_str_.c_str()),
    z_move(callback),
    &sub_options);
  zc_liveliness_subscriber_options_drop(z_move(sub_options));
  auto undeclare_z_sub = rcpputils::make_scope_exit(
    [this]() {
      z_undeclare_subscriber(z_move(this->graph_subscriber_));
    });
  if (!z_check(graph_subscriber_)) {
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
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return RMW_RET_OK;
  }

  z_undeclare_subscriber(z_move(graph_subscriber_));
  if (shm_manager_.has_value()) {
    z_drop(z_move(shm_manager_.value()));
  }
  // Close the zenoh session
  if (z_close(z_move(session_)) < 0) {
    RMW_SET_ERROR_MSG("Error while closing zenoh session");
    return RMW_RET_ERROR;
  }
  is_shutdown_ = true;
  return RMW_RET_OK;
}

///=============================================================================
rmw_context_impl_s::Data::~Data()
{
  RMW_TRY_DESTRUCTOR(
    static_cast<rmw_zenoh_cpp::GuardCondition *>(
      graph_guard_condition_->data)->~GuardCondition(),
    rmw_zenoh_cpp::GuardCondition, );
  if (rcutils_allocator_is_valid(allocator_)) {
    allocator_->deallocate(graph_guard_condition_->data, allocator_->state);
    allocator_->deallocate(graph_guard_condition_, allocator_->state);
    graph_guard_condition_ = nullptr;
  }

  auto ret = this->shutdown();
  static_cast<void>(ret);
}

///=============================================================================
rmw_context_impl_s::rmw_context_impl_s(
  const rcutils_allocator_t * allocator,
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
  z_owned_str_t shm_enabled = zc_config_get(z_loan(config), "transport/shared_memory/enabled");
  auto always_free_shm_enabled = rcpputils::make_scope_exit(
    [&shm_enabled]() {
      z_drop(z_move(shm_enabled));
    });

  // Initialize the zenoh session.
  z_owned_session_t session = z_open(z_move(config));
  if (!z_session_check(&session)) {
    throw std::runtime_error("Error setting up zenoh session");
  }
  auto close_session = rcpputils::make_scope_exit(
    [&session]() {
      z_close(z_move(session));
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
      std::this_thread::sleep_for(std::chrono::seconds(1));
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
  z_owned_reply_channel_t channel = zc_reply_fifo_new(0);
  zc_liveliness_get(
    z_loan(session), z_keyexpr(liveliness_str.c_str()),
    z_move(channel.send), NULL);
  z_owned_reply_t reply = z_reply_null();
  for (bool call_success = z_call(channel.recv, &reply); !call_success || z_check(reply);
    call_success = z_call(channel.recv, &reply))
  {
    if (!call_success) {
      continue;
    }
    if (z_reply_is_ok(&reply)) {
      z_sample_t sample = z_reply_ok(&reply);
      z_owned_str_t keystr = z_keyexpr_to_string(sample.keyexpr);
      // Ignore tokens from the same session to avoid race conditions from this
      // query and the liveliness subscription.
      graph_cache->parse_put(z_loan(keystr), true);
      z_drop(z_move(keystr));
    } else {
      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "[rmw_context_impl_s] z_call received an invalid reply\n");
    }
  }
  z_drop(z_move(reply));
  z_drop(z_move(channel));

  // Initialize the shm manager if shared_memory is enabled in the config.
  std::optional<zc_owned_shm_manager_t> shm_manager = std::nullopt;
  if (shm_enabled._cstr != nullptr &&
    strcmp(shm_enabled._cstr, "true") == 0)
  {
    char idstr[sizeof(zid.id) * 2 + 1];  // 2 bytes for each byte of the id, plus the trailing \0
    static constexpr size_t max_size_of_each = 3;  // 2 for each byte, plus the trailing \0
    for (size_t i = 0; i < sizeof(zid.id); ++i) {
      snprintf(idstr + 2 * i, max_size_of_each, "%02x", zid.id[i]);
    }
    idstr[sizeof(zid.id) * 2] = '\0';
    // TODO(yadunund): Can we get the size of the shm from the config even though it's not
    // a standard parameter?
    shm_manager =
      zc_shm_manager_new(
      z_loan(session),
      idstr,
      SHM_BUFFER_SIZE_MB * 1024 * 1024);
    if (!shm_manager.has_value() ||
      !zc_shm_manager_check(&shm_manager.value()))
    {
      throw std::runtime_error("Unable to create shm manager.");
    }
  }
  auto free_shm_manager = rcpputils::make_scope_exit(
    [&shm_manager]() {
      if (shm_manager.has_value()) {
        z_drop(z_move(shm_manager.value()));
      }
    });

  // Initialize the guard condition.
  rmw_guard_condition_t * graph_guard_condition =
    static_cast<rmw_guard_condition_t *>(allocator->zero_allocate(
      1, sizeof(rmw_guard_condition_t), allocator->state));
  if (graph_guard_condition == NULL) {
    throw std::runtime_error("failed to allocate graph guard condition");
  }
  auto free_guard_condition = rcpputils::make_scope_exit(
    [graph_guard_condition, allocator]() {
      allocator->deallocate(graph_guard_condition, allocator->state);
    });
  graph_guard_condition->implementation_identifier =
    rmw_zenoh_cpp::rmw_zenoh_identifier;
  graph_guard_condition->data =
    allocator->zero_allocate(1, sizeof(rmw_zenoh_cpp::GuardCondition), allocator->state);
  if (graph_guard_condition->data == NULL) {
    throw std::runtime_error("failed to allocate graph guard condition data");
  }
  auto free_guard_condition_data = rcpputils::make_scope_exit(
    [graph_guard_condition, allocator]() {
      allocator->deallocate(graph_guard_condition->data, allocator->state);
    });
  RMW_TRY_PLACEMENT_NEW(
    graph_guard_condition->data,
    graph_guard_condition->data,
    throw std::runtime_error("failed to initialize graph_guard_condition->data."),
    rmw_zenoh_cpp::GuardCondition);
  auto destruct_guard_condition_data = rcpputils::make_scope_exit(
    [graph_guard_condition]() {
      auto gc_data =
      static_cast<rmw_zenoh_cpp::GuardCondition *>(graph_guard_condition->data);
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        gc_data->~GuardCondition(),
        rmw_zenoh_cpp::GuardCondition);
    });

  close_session.cancel();
  free_shm_manager.cancel();
  destruct_guard_condition_data.cancel();
  free_guard_condition_data.cancel();
  free_guard_condition.cancel();

  data_ = std::make_shared<Data>(
    allocator,
    std::move(enclave),
    std::move(session),
    std::move(shm_manager),
    std::move(liveliness_str),
    std::move(graph_cache),
    graph_guard_condition);

  ret = data_->subscribe();
  if (ret != RMW_RET_OK) {
    throw std::runtime_error("Unable to subscribe to ROS Graph updates.");
  }
}

///=============================================================================
std::string rmw_context_impl_s::enclave() const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->enclave_;
}

///=============================================================================
z_session_t rmw_context_impl_s::session() const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return z_loan(data_->session_);
}

///=============================================================================
std::optional<zc_owned_shm_manager_t> & rmw_context_impl_s::shm_manager()
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->shm_manager_;
}

///=============================================================================
rmw_guard_condition_t * rmw_context_impl_s::graph_guard_condition()
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_guard_condition_;
}

///=============================================================================
size_t rmw_context_impl_s::get_next_entity_id()
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
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
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->is_shutdown_;
}

///=============================================================================
bool rmw_context_impl_s::session_is_valid() const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return z_check(data_->session_);
}

///=============================================================================
std::shared_ptr<rmw_zenoh_cpp::GraphCache> rmw_context_impl_s::graph_cache()
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_;
}
