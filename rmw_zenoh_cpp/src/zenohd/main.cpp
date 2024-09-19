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

#ifdef _WIN32
#include <windows.h>
#else
#include <signal.h>
#endif

#include <zenoh.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../detail/zenoh_config.hpp"
#include "../detail/liveliness_utils.hpp"

#include "rmw/error_handling.h"

#include "rcpputils/scope_exit.hpp"

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

  // If not already defined, set the logging environment variable for Zenoh router
  // to info level by default.
  // TODO(Yadunund): Switch to rcutils_get_env once it supports not overwriting values.
  if (setenv(ZENOH_LOG_ENV_VAR_STR, ZENOH_LOG_INFO_LEVEL_STR, 0) != 0) {
    RMW_SET_ERROR_MSG("Error configuring Zenoh logging.");
    return 1;
  }

  // Enable the zenoh built-in logger
  zc_try_init_log_from_env();

  // Initialize the zenoh configuration for the router.
  z_owned_config_t config;
  if ((rmw_zenoh_cpp::get_z_config(
      rmw_zenoh_cpp::ConfigurableEntity::Router,
      &config)) != RMW_RET_OK)
  {
    RMW_SET_ERROR_MSG("Error configuring Zenoh router.");
    return 1;
  }

  z_owned_session_t session;
  if (z_open(&session, z_move(config), NULL) != Z_OK) {
    printf("Unable to open router session!\n");
    return 1;
  }
  auto always_close_session = rcpputils::make_scope_exit(
    [&session]() {
      z_close(z_move(session), NULL);
    });

  printf(
    "Started Zenoh router with id %s.\n",
    rmw_zenoh_cpp::liveliness::zid_to_str(z_info_zid(z_session_loan(&session))).c_str());
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
