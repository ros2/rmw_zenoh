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

#ifndef ZENOH_SECURITY_CONFIGURATION_TOOLS__POLICY_PARSER_HPP_
#define ZENOH_SECURITY_CONFIGURATION_TOOLS__POLICY_PARSER_HPP_

#include <tinyxml2.h>

#include <cstdint>
#include <set>
#include <string>

#include <zenoh.hxx>

#include "zenoh_security_configuration_tools/visibility_control.hpp"

namespace zenoh
{
/**
 * This class parses the ROS 2 secutiry policy files into json5 Zenoh Config files
 **/
class PolicyParser
{
public:
  /// The library is loaded in the constructor.
  /**
   * \param[in] filename The policy string path.
   * \throws std::runtime_error if there are some invalid arguments or the library
   * was not load properly
   */
  ZENOH_SECURITY_CONFIGURATION_TOOLS_PUBLIC
  PolicyParser(
    const std::string & filename,
    const std::string & configfile,
    uint16_t domain_id);

  void parse();

private:
  void parse_enclaves(const tinyxml2::XMLElement * root);
  void parse_profiles(const tinyxml2::XMLElement * root);
  void parse_services(const tinyxml2::XMLElement * root, const std::string & node_name);
  void parse_topics(const tinyxml2::XMLElement * root, const std::string & node_name);
  void clear();
  void fill_data(
    zenoh::Config & config,
    const std::string & node_name);

  std::string check_name(
    const std::string & name,
    const std::string & node_name);

  tinyxml2::XMLDocument doc_;
  std::string configfile_path_;

  std::set<std::string> services_reply_allow_;
  std::set<std::string> services_reply_deny_;
  std::set<std::string> services_request_allow_;
  std::set<std::string> services_request_deny_;

  std::set<std::string> topics_sub_allow_;
  std::set<std::string> topics_pub_allow_;
  std::set<std::string> topics_sub_deny_;
  std::set<std::string> topics_pub_deny_;

  uint16_t domain_id_;
};
}  // namespace zenoh

#endif  // ZENOH_SECURITY_CONFIGURATION_TOOLS__POLICY_PARSER_HPP_
