// Copyright (c) 2025, Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <optional>
#include <sstream>

#include "config_generator.hpp"

namespace
{
//==============================================================================
struct CommandLineArgs
{
  bool help = false;
  std::optional<std::string> policy_filepath;
  std::optional<std::string> enclaves_dir;
  std::optional<uint8_t> ros_domain_id;
  std::optional<std::string> zenoh_session_config_filepath;
  std::optional<std::string> zenoh_router_config_filepath;
};

//==============================================================================
void print_help()
{
  std::cout << "Usage: ros2 run zenoh_security_tools generate_configs [options]\n\n"
            << "Generate Zenoh session and router configs with security artifacts.\n\n"
            << "Options:\n"
            << "  -h,--help                         Print this help message and exit\n"
            << "  -p,--policy TEXT REQUIRED         The path to the Access Control Policy file.\n"
            << "  -e,--enclaves TEXT                The directory with the security enclaves "
            << "for the various nodes in the policy file.\n"
            << "  -d,--ros-domain-id UINT REQUIRED  The ROS Domain ID.\n"
            << "  -c,--session-config TEXT REQUIRED The path to the Zenoh session config file.\n"
            << "  -r,--router-config TEXT REQUIRED  The path to the Zenoh router config file.\n"
            << std::endl;
}

//==============================================================================
std::optional<uint32_t> parse_uint(const std::string & s)
{
  try {
    size_t pos = 0;
    uint32_t n = std::stoul(s, &pos);
    if (pos == s.length()) {
      return n;
    }
  } catch (const std::invalid_argument &) {
    return {};
  } catch (const std::out_of_range &) {
    return {};
  }
  return {};
}

}  // namespace

//==============================================================================
int main(int argc, char *argv[])
{
  CommandLineArgs args;
  std::vector<std::string> raw_args(argv + 1, argv + argc);

  for (size_t i = 0; i < raw_args.size(); ++i) {
    const std::string & arg = raw_args[i];

    if (arg == "-h" || arg == "--help") {
      args.help = true;
    } else if ((arg == "-p" || arg == "--policy") && i + 1 < raw_args.size()) {
      args.policy_filepath = raw_args[++i];
    } else if ((arg == "-e" || arg == "--enclaves") && i + 1 < raw_args.size()) {
      args.enclaves_dir = raw_args[++i];
    } else if ((arg == "-d" || arg == "--ros-domain-id") && i + 1 < raw_args.size()) {
      auto value = parse_uint(raw_args[++i]);
      if (value) {
        args.ros_domain_id = value.value();
      } else {
        std::cerr << "Error: Invalid value for --ros-domain-id: " << raw_args[i] << std::endl;
        print_help();
        return 1;
      }
    } else if ((arg == "-c" || arg == "--session-config") && i + 1 < raw_args.size()) {
      args.zenoh_session_config_filepath = raw_args[++i];
    } else if ((arg == "-r" || arg == "--router-config") && i + 1 < raw_args.size()) {
      args.zenoh_router_config_filepath = raw_args[++i];
    } else {
      std::cerr << "Error: Unknown option: " << arg << std::endl;
      print_help();
      return 1;
    }
  }

  if (args.help) {
    print_help();
    return 0;
  }

  if (!args.policy_filepath) {
    std::cerr << "Error: --policy is required." << std::endl;
    print_help();
    return 1;
  }

  if (!args.ros_domain_id) {
    std::cerr << "Error: --ros-domain-id is required." << std::endl;
    print_help();
    return 1;
  }

  if (!args.zenoh_session_config_filepath) {
    std::cerr << "Error: --session-config is required." << std::endl;
    print_help();
    return 1;
  }

  if (!args.zenoh_router_config_filepath) {
    std::cerr << "Error: --router-config is required." << std::endl;
    print_help();
    return 1;
  }

  auto config_generator = zenoh_security_tools::ConfigGenerator(
    args.policy_filepath.value(),
    args.enclaves_dir.value(),
    args.zenoh_router_config_filepath.value(),
    args.zenoh_session_config_filepath.value(),
    args.ros_domain_id.value());
  config_generator.generate();
  return 0;
}
