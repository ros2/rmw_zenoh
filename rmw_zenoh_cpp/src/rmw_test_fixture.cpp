// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include <rmw_test_fixture/rmw_test_fixture.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>
#include <rcpputils/env.hpp>
#include <zenoh.hxx>
#include <zenoh/api/session.hxx>

#include "detail/zenoh_config.hpp"

static std::unique_ptr<zenoh::Session> g_session = nullptr;

static
std::optional<std::string>
get_endpoints(zenoh::Session & session)
{
  const zenoh::Id zid = session.get_zid();
  const zenoh::KeyExpr keyexpr("@/" + zid.to_string() + "/router");

  zenoh::ZResult result;
  const zenoh::channels::FifoChannel::HandlerType<zenoh::Reply> replies = session.get(
    keyexpr,
    "",
    zenoh::channels::FifoChannel(2),
    zenoh::Session::GetOptions::create_default(),
    &result);
  if (Z_OK != result) {
    std::cerr << "Error calling get over keyexpr "
              << std::string(keyexpr.as_string_view()) << std::endl;
    return std::nullopt;
  }

  const std::variant<zenoh::Reply, zenoh::channels::RecvError> reply = replies.recv();
  const zenoh::Sample & sample = std::get<zenoh::Reply>(reply).get_ok();
  nlohmann::json parsed;

  try {
    parsed = nlohmann::json::parse(sample.get_payload().as_string());
  } catch (nlohmann::json::exception & e) {
    std::cerr << "Failed to parse admin space response: " << e.what() << std::endl;
    return std::nullopt;
  }

  return parsed["locators"].dump();
}

extern "C"
{
/// Isolate Zenoh communication using an ad-hoc router
/**
 * This fixture creates a new Zenoh router on a random unused port number for
 * use by the current process. The router does not connect to other routers,
 * but does respect other Zenoh configurations provided by configuration files
 * and environment variables.
 *
 * After calling this function, the ZENOH_CONFIG_OVERRIDE environment variable
 * for this process will configure Zenoh to use the ad-hoc router using the
 * `connect/endpoints` configuration key, which is populated from the
 * `listen/endpoints` configuration of the router.
 *
 * Calling `rmw_test_isolation_stop()` will gracefully shut down the router.
 */
rmw_ret_t
rmw_test_isolation_start()
{
  zenoh::ZResult result;

  zenoh::try_init_log_from_env();

  std::optional<zenoh::Config> config = rmw_zenoh_cpp::get_z_config(
    rmw_zenoh_cpp::ConfigurableEntity::Router);

  if (!config.has_value()) {
    std::cerr << "Error configuring Zenoh router." << std::endl;
    return RMW_RET_ERROR;
  }

  config->insert_json5("listen/endpoints", "[\"tcp/127.0.0.1:0\"]", &result);
  if (result != Z_OK) {
    std::cerr << "Error setting router endpoint" << std::endl;
    return RMW_RET_ERROR;
  }

  config->insert_json5("connect/endpoints", "[]", &result);
  if (result != Z_OK) {
    std::cerr << "Error setting router connect endpoints" << std::endl;
    return RMW_RET_ERROR;
  }

  config->insert_json5("scouting/multicast/enabled", "false", &result);
  if (result != Z_OK) {
    std::cerr << "Error setting router multicast disabled" << std::endl;
    return RMW_RET_ERROR;
  }

  g_session = std::make_unique<zenoh::Session>(
    std::move(config.value()),
    zenoh::Session::SessionOptions::create_default(),
    &result);
  if (result != Z_OK) {
    std::cerr << "Error opening Session!" << std::endl;
    return RMW_RET_ERROR;
  }

  std::optional<std::string> endpoints = get_endpoints(*g_session);
  if (!endpoints.has_value()) {
    // Error already logged.
    return RMW_RET_ERROR;
  }

  std::string config_override = "connect/endpoints=" + endpoints.value();
  if (!rcpputils::set_env_var(
      "ZENOH_CONFIG_OVERRIDE",
      config_override.c_str()))
  {
    std::cerr << "Failed to set ZENOH_CONFIG_OVERRIDE" << std::endl;
    g_session->close();
    g_session.reset();
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_test_isolation_stop()
{
  rcpputils::set_env_var("ZENOH_CONFIG_OVERRIDE", nullptr);

  if(g_session) {
    g_session->close();
    g_session.reset();
  }

  return RMW_RET_OK;
}
}
