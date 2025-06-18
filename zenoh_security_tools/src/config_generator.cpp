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

#include "config_generator.hpp"

#include <tinyxml2.h>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <nlohmann/json.hpp>

#include "rcpputils/scope_exit.hpp"
#include "rcutils/allocator.h"
#include "rcutils/types/string_map.h"
#include "rmw_dds_common/security.hpp"

#include <zenoh.hxx>

static const char * root_str = "policy";
static const char * enclaves_str = "enclaves";
static const char * enclave_str = "enclave";
static const char * profiles_str = "profiles";
static const char * profile_str = "profile";
static const char * router_str = "zenohd";
static const char * services_str = "services";
static const char * service_str = "service";
static const char * topics_str = "topics";
static const char * topic_str = "topic";

using json = nlohmann::json;

namespace zenoh_security_tools
{
//==============================================================================
ConfigGenerator::ConfigGenerator(
  const std::string & policy_filepath,
  const std::string & enclaves_dir,
  const std::string & zenoh_router_config_filepath,
  const std::string & zenoh_session_config_filepath,
  uint8_t domain_id)
: enclaves_dir_(std::nullopt),
  zenoh_router_config_filepath_(std::move(zenoh_router_config_filepath)),
  zenoh_session_config_filepath_(std::move(zenoh_session_config_filepath)),
  domain_id_(std::move(domain_id))
{
  const tinyxml2::XMLError error = doc_.LoadFile(policy_filepath.c_str());
  if (error != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Invalid argument: wrong policy file.");
  }

  if (!enclaves_dir.empty()) {
    std::filesystem::path maybe_dir{enclaves_dir};
    if (std::filesystem::is_directory(enclaves_dir)) {
      enclaves_dir_ = std::move(maybe_dir);
    }
  }
}

namespace
{
//==============================================================================
bool replace(
  std::string & str,
  const std::string & from,
  const std::string & to)
{
  size_t start_pos = str.find(from);
  if(start_pos == std::string::npos) {
    return false;
  }
  str.replace(start_pos, from.length(), to);
  return true;
}

//==============================================================================
json to_key_exprs(
  const std::set<std::string> & key_exprs,
  uint8_t domain_id)
{
  json key_exprs_ret = json::array();

  for (const auto & name : key_exprs) {
    key_exprs_ret.push_back(std::to_string(domain_id) + "/" + name + "/**");
  }

  return key_exprs_ret;
}

//==============================================================================
std::string check_name(
  const std::string & name,
  const std::string & node_name)
{
  std::string result = name;
  replace(result, "~", node_name);
  if (result[0] == '/') {
    result = result.substr(1);
  }
  return result;
}
}  // anonymous namespace

//==============================================================================
void ConfigGenerator::parse_services(
  const tinyxml2::XMLElement * root,
  const std::string & node_name)
{
  const tinyxml2::XMLElement * services_node = root->FirstChildElement();
  do{
    if (services_node != nullptr) {
      if (strcmp(services_node->Name(), services_str) == 0) {
        std::string service_type;
        const char * permission_s = services_node->Attribute("reply");
        if (permission_s != nullptr) {
          service_type = "reply";
        } else {
          permission_s = services_node->Attribute("request");
          if (permission_s != nullptr) {
            service_type = "request";
          }
        }

        if (permission_s == nullptr) {
          throw std::runtime_error("Not able to get permission from service " +
            std::to_string(services_node->GetLineNum()));
        }
        std::string permission = permission_s;

        const tinyxml2::XMLElement * service_node = services_node->FirstChildElement();
        do {
          if (service_node != nullptr) {
            if (strcmp(service_node->Name(), service_str) == 0) {
              if (service_type == "reply") {
                if (permission == "ALLOW") {
                  services_reply_allow_.insert(check_name(service_node->GetText(), node_name));
                } else if (permission == "DENY") {
                  services_reply_deny_.insert(check_name(service_node->GetText(), node_name));
                }
              } else if (service_type == "request") {
                if (permission == "ALLOW") {
                  services_request_allow_.insert(check_name(service_node->GetText(), node_name));
                } else if (permission == "DENY") {
                  services_request_deny_.insert(check_name(service_node->GetText(), node_name));
                }
              }
            } else {
              throw std::runtime_error("Invalid file");
            }
          }
        } while ((service_node = service_node->NextSiblingElement()) != nullptr);
      }
    } else {
      throw std::runtime_error("Invalid file");
    }
  } while ((services_node = services_node->NextSiblingElement()) != nullptr);
}

//==============================================================================
void ConfigGenerator::clear()
{
  services_reply_allow_.clear();
  services_reply_deny_.clear();
  services_request_allow_.clear();
  services_request_deny_.clear();
  topics_sub_allow_.clear();
  topics_pub_allow_.clear();
  topics_sub_deny_.clear();
  topics_pub_deny_.clear();
}

//==============================================================================
void ConfigGenerator::fill_access_control(
  zenoh::Config & config,
  const std::string & node_name)
{
  json rules = json::array();
  json policies_rules = json::array();

  if (!services_reply_allow_.empty()) {
    json rule_allow_reply = json::object({
        {"id", "incoming_queries"},
        {"messages", json::array({"query"})},
        {"flows", json::array({"ingress"})},
        {"permission", "allow"},
        {"key_exprs", to_key_exprs(services_reply_allow_, domain_id_)},
    });
    rules.push_back(rule_allow_reply);
    policies_rules.push_back("incoming_queries");

    json rule_outgoing_reply = json::object({
        {"id", "outgoing_queryables_replies"},
        {"messages", json::array({"declare_queryable", "reply"})},
        {"flows", json::array({"egress"})},
        {"permission", "allow"},
        {"key_exprs", to_key_exprs(services_reply_allow_, domain_id_)},
    });
    rules.push_back(rule_outgoing_reply);
    policies_rules.push_back("outgoing_queryables_replies");
  }

  if (!services_request_allow_.empty()) {
    json rule_allow_request_out = json::object({
        {"id", "outgoing_queries"},
        {"messages", json::array({"query"})},
        {"flows", json::array({"egress"})},
        {"permission", "allow"},
        {"key_exprs", to_key_exprs(services_request_allow_, domain_id_)},
    });
    rules.push_back(rule_allow_request_out);
    policies_rules.push_back("outgoing_queries");

    json rule_allow_request_in = json::object({
        {"id", "incoming_queryables_replies"},
        {"messages", json::array({"declare_queryable", "reply"})},
        {"flows", json::array({"ingress"})},
        {"permission", "allow"},
        {"key_exprs", to_key_exprs(services_request_allow_, domain_id_)},
    });
    rules.push_back(rule_allow_request_in);
    policies_rules.push_back("incoming_queryables_replies");
  }

  if (!topics_pub_allow_.empty()) {
    json rule_allow_pub_out = json::object({
        {"id", "outgoing_publications"},
        {"messages", json::array({"put"})},
        {"flows", json::array({"egress"})},
        {"permission", "allow"},
        {"key_exprs", to_key_exprs(topics_pub_allow_, domain_id_)},
    });
    rules.push_back(rule_allow_pub_out);
    policies_rules.push_back("outgoing_publications");

    json rule_allow_pub_in = json::object({
        {"id", "incoming_subscriptions"},
        {"messages", json::array({"declare_subscriber"})},
        {"flows", json::array({"ingress"})},
        {"permission", "allow"},
        {"key_exprs", to_key_exprs(topics_pub_allow_, domain_id_)},
    });
    rules.push_back(rule_allow_pub_in);
    policies_rules.push_back("incoming_subscriptions");
  }

  if (!topics_sub_allow_.empty()) {
    json rule_allow_sub_out = json::object({
        {"id", "outgoing_subscriptions"},
        {"messages", json::array({"declare_subscriber"})},
        {"flows", json::array({"egress"})},
        {"permission", "allow"},
        {"key_exprs", to_key_exprs(topics_sub_allow_, domain_id_)},
    });
    rules.push_back(rule_allow_sub_out);
    policies_rules.push_back("outgoing_subscriptions");

    json rule_allow_sub_in = json::object({
        {"id", "incoming_publications"},
        {"messages", json::array({"put"})},
        {"flows", json::array({"ingress"})},
        {"permission", "allow"},
        {"key_exprs", to_key_exprs(topics_sub_allow_, domain_id_)},
    });
    rules.push_back(rule_allow_sub_in);
    policies_rules.push_back("incoming_publications");
  }

  json liveliness_messages = json::array({
      "liveliness_token", "liveliness_query", "declare_liveliness_subscriber"});
  if (!services_reply_allow_.empty() || !services_request_allow_.empty()) {
    liveliness_messages.push_back("reply");
  }

  json rule_liveliness = json::object({
      {"id", "liveliness_tokens"},
      {"messages", liveliness_messages},
      {"flows", json::array({"ingress", "egress"})},
      {"permission", "allow"},
      {"key_exprs",
        json::array({"@ros2_lv/" + std::to_string(domain_id_) + "/**"})},
  });
  rules.push_back(rule_liveliness);
  policies_rules.push_back("liveliness_tokens");

  json policies = json::array();
  policies.push_back(json::object({
      {"rules", json::array({"liveliness_tokens"})},
      {"subjects", json::array({"router"})},
  }));
  policies.push_back(json::object({
      {"rules", policies_rules},
      {"subjects", json::array({node_name})},
  }));

  json subjects = json::array({
      json::object({{"id", "router"}}),
      json::object({{"id", node_name}}),
  });

  config.insert_json5("access_control/enabled", "true");
  config.insert_json5("access_control/default_permission", "'deny'");
  config.insert_json5("access_control/rules", rules.dump());
  config.insert_json5("access_control/policies", policies.dump());
  config.insert_json5("access_control/subjects", subjects.dump());
}

//==============================================================================
void ConfigGenerator::fill_certificates(
  zenoh::Config & config,
  const std::string & node_name)
{
  // Skip this step if enclaves directory was not specified.
  if (!enclaves_dir_.has_value()) {
    return;
  }
  auto enclaves_dir = enclaves_dir_.value();
  auto enclave_dir = enclaves_dir / node_name;
  if (!std::filesystem::is_directory(enclaves_dir)) {
    std::cout << "No directory with name "
              << node_name
              << " present within enclaves directory "
              << enclaves_dir.string()
              << ". Skipping authentication..."
              << std::endl;
    return;
  }

  // Access the certificates using utility function from rmw_dds_common.
  std::unordered_map<std::string, std::string> security_files;
  if (!rmw_dds_common::get_security_files(
      false, std::string(""), enclave_dir.string(), security_files))
  {
    std::cerr << "Failed to get certificates for " << node_name << " from" <<
      enclave_dir.string().c_str() << std::endl;
    return;
  }

  // TODO(Yadunund): Actually check if some of these configs are already set and only update
  // their values as opposed to overwriting.
  try {
    json tls_config_json = {
      {"link", {
          {"protocols", json::array({"tls"})},
          {"tls", {
              {"enable_mtls", true},
              {"verify_name_on_connect", false},
              {"root_ca_certificate", security_files["IDENTITY_CA"]},
              {"listen_private_key", security_files["PRIVATE_KEY"]},
              {"listen_certificate", security_files["CERTIFICATE"]},
              {"connect_private_key", security_files["PRIVATE_KEY"]},
              {"connect_certificate", security_files["CERTIFICATE"]}
            }}
        }}
    };
    // Insert the config.
    config.insert_json5("transport", tls_config_json.dump());
  } catch (const std::exception & e) {
    std::cerr << "Error creating tls_config_json: " << e.what() << std::endl;
    return;
  }

  auto replace_with_tls =
    [](const std::string & key, zenoh::Config & config) -> void
    {
      try {
        const json endpoints = json::parse(config.get(key));
        json tls_endpoints_json = json::array();
        for (const json & endpoint : endpoints) {
          std::string endpoint_str = endpoint.get<std::string>();
          const std::size_t slash_pos = endpoint_str.find('/');
          if (slash_pos != std::string::npos) {
            tls_endpoints_json.push_back(endpoint_str.replace(0, slash_pos, "tls"));
          }
        }
        config.insert_json5(key, tls_endpoints_json.dump());
      } catch (const std::exception & e) {
        std::cerr << "Error replacing transport with tls: " << e.what() << std::endl;
        return;
      }
    };

  replace_with_tls("connect/endpoints", config);
  replace_with_tls("listen/endpoints", config);
}

//==============================================================================
void ConfigGenerator::parse_topics(
  const tinyxml2::XMLElement * root,
  const std::string & node_name)
{
  const tinyxml2::XMLElement * topics_node = root->FirstChildElement();
  do{
    if (topics_node != nullptr) {
      if (strcmp(topics_node->Name(), topics_str) == 0) {
        std::string topic_type;
        const char * permission_s = topics_node->Attribute("subscribe");
        if (permission_s != nullptr) {
          topic_type = "subscribe";
        } else {
          permission_s = topics_node->Attribute("publish");
          if (permission_s != nullptr) {
            topic_type = "publish";
          }
        }

        if (permission_s == nullptr) {
          throw std::runtime_error("Not able to get permission from service " +
            std::to_string(topics_node->GetLineNum()));
        }
        std::string permission = permission_s;

        const tinyxml2::XMLElement * topic_node = topics_node->FirstChildElement();
        do {
          if (topic_node != nullptr) {
            if (strcmp(topic_node->Name(), topic_str) == 0) {
              if (topic_type == "publish") {
                if (permission == "ALLOW") {
                  topics_pub_allow_.insert(check_name(topic_node->GetText(), node_name));
                } else if (permission == "DENY") {
                  topics_pub_allow_.insert(check_name(topic_node->GetText(), node_name));
                }
              } else if (topic_type == "subscribe") {
                if (permission == "ALLOW") {
                  topics_sub_allow_.insert(check_name(topic_node->GetText(), node_name));
                } else if (permission == "DENY") {
                  topics_sub_deny_.insert(check_name(topic_node->GetText(), node_name));
                }
              }

            } else {
              throw std::runtime_error("Invalid file");
            }
          }
        } while ((topic_node = topic_node->NextSiblingElement()) != nullptr);
      }
    } else {
      throw std::runtime_error("Invalid file");
    }
  } while ((topics_node = topics_node->NextSiblingElement()) != nullptr);
}

//==============================================================================
void ConfigGenerator::parse_profiles(const tinyxml2::XMLElement * root)
{
  const tinyxml2::XMLElement * profiles_node = root->FirstChildElement();
  do{
    if (profiles_node != nullptr) {
      if (strcmp(profiles_node->Name(), profiles_str) == 0) {
        const tinyxml2::XMLElement * profile_node = profiles_node->FirstChildElement();
        do {
          if (profile_node != nullptr) {
            if (strcmp(profile_node->Name(), profile_str) == 0) {
              const char * node_name = profile_node->Attribute("node");
              if (node_name == nullptr) {
                std::string error_msg = "Attribute name is required in " +
                  std::string(profile_str) + " tag. Line " +
                  std::to_string(profiles_node->GetLineNum());
                throw std::runtime_error(error_msg);
              }

              zenoh::ZResult result;
              zenoh::Config config = zenoh::Config::from_file(zenoh_session_config_filepath_,
                  &result);
              if (result != Z_OK) {
                std::string error_msg = "Invalid configuration file " +
                  zenoh_session_config_filepath_;
                throw std::runtime_error("Error getting Zenoh session config file.");
              }

              parse_services(profile_node, node_name);
              parse_topics(profile_node, node_name);

              this->fill_access_control(config, node_name);
              this->fill_certificates(config, node_name);

              std::string filename = std::string(node_name) + ".json5";
              std::ofstream new_config_file(filename);
              json j_config = json::parse(config.to_string());
              new_config_file << j_config.dump(2);
              std::cout << "New file create called " << filename << std::endl;
              new_config_file.close();

              this->clear();
            }
          } else {
            throw std::runtime_error("Invalid file");
          }
        } while ((profile_node = profile_node->NextSiblingElement()) != nullptr);
      } else {
        std::string error_msg = "Invalid file: Malformed Zenoh policy root. Line: " +
          std::to_string(profiles_node->GetLineNum());
        throw std::runtime_error(error_msg);
      }
    } else {
      throw std::runtime_error("Invalid file");
    }
  } while ((profiles_node = profiles_node->NextSiblingElement()) != nullptr);
}

//==============================================================================
void ConfigGenerator::parse_enclaves(const tinyxml2::XMLElement * root)
{
  const tinyxml2::XMLElement * enclaves_node = root->FirstChildElement();
  if (enclaves_node != nullptr) {
    if (strcmp(enclaves_node->Name(), enclaves_str) == 0) {
      const tinyxml2::XMLElement * enclave_node = enclaves_node->FirstChildElement();
      if (enclave_node != nullptr) {
        if (strcmp(enclave_node->Name(), enclave_str) == 0) {
          parse_profiles(enclave_node);
        }
      } else {
        throw std::runtime_error("Invalid file");
      }
    } else {
      std::string error_msg = "Invalid file: Malformed Zenoh policy root. Line: " +
        std::to_string(enclaves_node->GetLineNum());
      throw std::runtime_error(error_msg);
    }
  } else {
    throw std::runtime_error("Invalid file");
  }
}

//==============================================================================
void ConfigGenerator::generate_router_config()
{
  zenoh::ZResult result;
  zenoh::Config config = zenoh::Config::from_file(zenoh_router_config_filepath_, &result);
  if (result != Z_OK) {
    std::string error_msg = "Invalid configuration file " + zenoh_session_config_filepath_;
    throw std::runtime_error("Error getting Zenoh router config file.");
  }

  this->fill_certificates(config, router_str);

  std::string filename = std::string(router_str) + ".json5";
  std::ofstream new_config_file(filename);
  json j_config = json::parse(config.to_string());
  new_config_file << j_config.dump(2);
  std::cout << "New file create called " << filename << std::endl;
  new_config_file.close();
}

//==============================================================================
void ConfigGenerator::generate_session_configs()
{
  const tinyxml2::XMLElement * root = doc_.RootElement();
  if (root != nullptr) {
    if (strcmp(root->Name(), root_str) == 0) {
      parse_enclaves(root);
    } else {
      std::string error_msg = "Invalid file: Malformed Zenoh policy root. Line: " +
        std::to_string(root->GetLineNum());
      throw std::runtime_error(error_msg);
    }
  } else {
    throw std::runtime_error("Invalid file");
  }
}

//==============================================================================
void ConfigGenerator::generate()
{
  generate_session_configs();
  generate_router_config();
}

}  // namespace zenoh_security_tools
