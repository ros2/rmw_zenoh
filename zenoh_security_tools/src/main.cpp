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

#include <string>
#include <cstdint>

#include <CLI/CLI.hpp>

#include "config_generator.hpp"

//==============================================================================
int main(int argc, char * argv[])
{
  CLI::App app{"Generate Zenoh session configs with security artifacts.\n"};

  std::string policy_filepath;
  std::string enclaves_dir;
  std::string zenoh_router_config_filepath;
  std::string zenoh_session_config_filepath;
  uint16_t domain_id = 0;
  app.add_option("-p,--policy", policy_filepath,
    "The path to the Access Control Policy file.")->required();
  app.add_option("-e,--enclaves", enclaves_dir,
    "The directory with the security enclaves for the various nodes in the policy file.");
  app.add_option("-d,--ros-domain-id", domain_id, "The ROS Domain ID.")->required();
  app.add_option("-c,--session-config", zenoh_session_config_filepath,
    "The path to the Zenoh session config file.")->required();
  app.add_option("-r,--router-config", zenoh_router_config_filepath,
    "The path to the Zenoh router config file.")->required();

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    return app.exit(e);
  }

  auto config_generator = zenoh_security_tools::ConfigGenerator(
    policy_filepath,
    enclaves_dir,
    zenoh_router_config_filepath,
    zenoh_session_config_filepath,
    domain_id);
  config_generator.generate();
  return 0;
}
