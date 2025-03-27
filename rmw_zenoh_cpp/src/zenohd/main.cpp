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

#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <stdexcept>

#include <zenoh.hxx>
#include <zenoh/api/session.hxx>

#ifdef _WIN32
#include <windows.h>
#else
#include <signal.h>
#endif

#include "../detail/zenoh_config.hpp"

#include "rmw/error_handling.h"
#include "rcutils/env.h"

static bool running = true;
static std::mutex run_mutex;
static std::condition_variable run_cv;

#ifdef _WIN32
BOOL WINAPI quit(DWORD ctrl_type)
{
  (void)ctrl_type;
  running = false;
  run_cv.notify_one();
  return true;
}
#else
void quit(int sig)
{
  (void)sig;
  running = false;
  run_cv.notify_one();
}
#endif

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  if (!rcutils_set_env_overwrite(ZENOH_LOG_ENV_VAR_STR, ZENOH_LOG_INFO_LEVEL_STR, 0)) {
    RMW_SET_ERROR_MSG("Error configuring Zenoh logging.");
    return 1;
  }

  // Enable the zenoh built-in logger
  zenoh::try_init_log_from_env();

  std::optional<zenoh::Config> config = rmw_zenoh_cpp::get_z_config(
    rmw_zenoh_cpp::ConfigurableEntity::Router);

  if (!config.has_value()) {
    RMW_SET_ERROR_MSG("Error configuring Zenoh router.");
    return 1;
  }

  zenoh::ZResult result;
  auto session = zenoh::Session::open(
    std::move(config.value()),
    zenoh::Session::SessionOptions::create_default(),
    &result);
  if (result != Z_OK) {
    std::cout << "Error opening Session!" << "\\n";
    return 1;
  }

  std::cout << "Started Zenoh router with id "
            << session.get_zid().to_string()
            << std::endl;

#ifdef _WIN32
  SetConsoleCtrlHandler(quit, TRUE);
#else
  signal(SIGINT, quit);
  signal(SIGTERM, quit);
#endif

  // Wait until it's time to exit.
  std::unique_lock lock(run_mutex);
  run_cv.wait(lock, [] {return !running;});

  return 0;
}
