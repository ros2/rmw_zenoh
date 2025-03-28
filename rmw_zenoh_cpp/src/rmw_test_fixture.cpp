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
#include <memory>
#include <sstream>

#include <nlohmann/json.hpp>
#include <rcpputils/env.hpp>
#include <zenoh.hxx>
#include <zenoh/api/session.hxx>

#include "detail/zenoh_config.hpp"

static std::unique_ptr<zenoh::Session> g_session = nullptr;

static
std::string
get_endpoints(zenoh::Session & session)
{
  auto zid = session.get_zid();
  auto keyexpr = zenoh::KeyExpr("@/" + zid.to_string() + "/router");
  auto replies = session.get(keyexpr, "", zenoh::channels::FifoChannel(2));

  auto reply = replies.recv();
  const auto & sample = std::get<zenoh::Reply>(reply).get_ok();
  auto parsed = nlohmann::json::parse(sample.get_payload().as_string());

  return parsed["locators"].dump();
}

extern "C"
{
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

  g_session = std::make_unique<zenoh::Session>(
    std::move(config.value()),
    zenoh::Session::SessionOptions::create_default(),
    &result);
  if (result != Z_OK) {
    std::cerr << "Error opening Session!" << std::endl;
    return RMW_RET_ERROR;
  }

  std::string endpoints = get_endpoints(*g_session);

  std::string config_override = "connect/endpoints=" + endpoints;
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
