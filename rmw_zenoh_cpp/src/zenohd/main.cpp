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

#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#else
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#endif

#include <zenoh.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../detail/zenoh_config.hpp"
#include "../detail/liveliness_utils.hpp"

#include "rmw/error_handling.h"

#include "rcpputils/scope_exit.hpp"

static bool running = true;

class KeyboardReader final
{
public:
  KeyboardReader()
  {
#ifdef _WIN32
    hstdin_ = GetStdHandle(STD_INPUT_HANDLE);
    if (hstdin_ == INVALID_HANDLE_VALUE) {
      throw std::runtime_error("Failed to get stdin handle");
    }
    if (!GetConsoleMode(hstdin_, &old_mode_)) {
      throw std::runtime_error("Failed to get old console mode");
    }
    DWORD new_mode = ENABLE_PROCESSED_INPUT;  // for Ctrl-C processing
    if (!SetConsoleMode(hstdin_, new_mode)) {
      throw std::runtime_error("Failed to set new console mode");
    }
#else
    // get the console in raw mode
    if (tcgetattr(0, &cooked_) < 0) {
      throw std::runtime_error("Failed to get old console mode");
    }
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    raw.c_cc[VTIME] = 1;
    raw.c_cc[VMIN] = 0;
    if (tcsetattr(0, TCSANOW, &raw) < 0) {
      throw std::runtime_error("Failed to set new console mode");
    }
#endif
  }

  char readOne()
  {
    char c = 0;

#ifdef _WIN32
    INPUT_RECORD record;
    DWORD num_read;
    switch (WaitForSingleObject(hstdin_, 100)) {
      case WAIT_OBJECT_0:
        if (!ReadConsoleInput(hstdin_, &record, 1, &num_read)) {
          throw std::runtime_error("Read failed");
        }

        if (record.EventType != KEY_EVENT || !record.Event.KeyEvent.bKeyDown) {
          break;
        }

        if (record.Event.KeyEvent.wVirtualKeyCode == VK_LEFT) {
          c = KEYCODE_LEFT;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == VK_UP) {
          c = KEYCODE_UP;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT) {
          c = KEYCODE_RIGHT;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == VK_DOWN) {
          c = KEYCODE_DOWN;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x42) {
          c = KEYCODE_B;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x43) {
          c = KEYCODE_C;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x44) {
          c = KEYCODE_D;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x45) {
          c = KEYCODE_E;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x46) {
          c = KEYCODE_F;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x47) {
          c = KEYCODE_G;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x51) {
          c = KEYCODE_Q;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x52) {
          c = KEYCODE_R;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x54) {
          c = KEYCODE_T;
        } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x56) {
          c = KEYCODE_V;
        }
        break;

      case WAIT_TIMEOUT:
        break;
    }

#else
    int rc = read(0, &c, 1);
    if (rc < 0) {
      throw std::runtime_error("read failed");
    }
#endif

    return c;
  }

  ~KeyboardReader()
  {
#ifdef _WIN32
    SetConsoleMode(hstdin_, old_mode_);
#else
    tcsetattr(0, TCSANOW, &cooked_);
#endif
  }

private:
#ifdef _WIN32
  HANDLE hstdin_;
  DWORD old_mode_;
#else
  struct termios cooked_;
#endif
};

#ifdef _WIN32
BOOL WINAPI quit(DWORD ctrl_type)
{
  (void)ctrl_type;
  running = false;
  return true;
}
#else
void quit(int sig)
{
  (void)sig;
  running = false;
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

  // Initialize the zenoh configuration for the router.
  z_owned_config_t config;
  if ((rmw_zenoh_cpp::get_z_config(
      rmw_zenoh_cpp::ConfigurableEntity::Router,
      &config)) != RMW_RET_OK)
  {
    RMW_SET_ERROR_MSG("Error configuring Zenoh router.");
    return 1;
  }

  z_owned_session_t session = z_open(z_move(config));
  if (!z_check(session)) {
    printf("Unable to open router session!\n");
    return 1;
  }
  auto close_session = rcpputils::make_scope_exit(
    [&session]() {
      z_close(z_move(session));
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

  KeyboardReader keyreader;

  char c = 0;

  printf("Enter 'q' to quit...\n");
  while (running) {
    // get the next event from the keyboard
    try {
      c = keyreader.readOne();
    } catch (const std::runtime_error &) {
      perror("read():");
      return -1;
    }

    if (c == 'q') {
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
