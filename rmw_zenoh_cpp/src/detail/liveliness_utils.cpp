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

#include "liveliness_utils.hpp"

#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcpputils/scope_exit.hpp"
#include "rcutils/logging_macros.h"


namespace liveliness
{

///=============================================================================
NodeInfo::NodeInfo(
  std::size_t domain_id,
  std::string ns,
  std::string name,
  std::string enclave)
: domain_id_(std::move(domain_id)),
  ns_(std::move(ns)),
  name_(std::move(name)),
  enclave_(std::move(enclave))
{
  // Do nothing.
}

///=============================================================================
TopicInfo::TopicInfo(
  std::string name,
  std::string type,
  rmw_qos_profile_t qos)
: name_(std::move(name)),
  type_(std::move(type)),
  qos_(std::move(qos))
{
  // Do nothing.
}

///=============================================================================
namespace
{

/// The admin space used to prefix the liveliness tokens.
static const char ADMIN_SPACE[] = "@ros2_lv";
static const char NODE_STR[] = "NN";
static const char PUB_STR[] = "MP";
static const char SUB_STR[] = "MS";
static const char SRV_STR[] = "SS";
static const char CLI_STR[] = "SC";
static const char SLASH_REPLACEMENT = '%';
static const char QOS_DELIMITER = ':';
static const char QOS_HISTORY_DELIMITER = ',';

static const std::unordered_map<EntityType, std::string> entity_to_str = {
  {EntityType::Node, NODE_STR},
  {EntityType::Publisher, PUB_STR},
  {EntityType::Subscription, SUB_STR},
  {EntityType::Service, SRV_STR},
  {EntityType::Client, CLI_STR}
};

static const std::unordered_map<std::string, EntityType> str_to_entity = {
  {NODE_STR, EntityType::Node},
  {PUB_STR, EntityType::Publisher},
  {SUB_STR, EntityType::Subscription},
  {SRV_STR, EntityType::Service},
  {CLI_STR, EntityType::Client}
};

std::string zid_to_str(z_id_t id)
{
  std::stringstream ss;
  ss << std::hex;
  size_t i = 0;
  for (; i < (sizeof(id.id)); i++) {
    ss << static_cast<int>(id.id[i]);
  }
  return ss.str();
}

std::vector<std::string> split_keyexpr(
  const std::string & keyexpr,
  const char delim = '/')
{
  std::vector<std::size_t> delim_idx = {};
  // Insert -1 for starting position to make the split easier when using substr.
  delim_idx.push_back(-1);
  std::size_t idx = 0;
  for (std::string::const_iterator it = keyexpr.begin(); it != keyexpr.end(); ++it) {
    if (*it == delim) {
      delim_idx.push_back(idx);
    }
    ++idx;
  }
  std::vector<std::string> result = {};
  try {
    for (std::size_t i = 1; i < delim_idx.size(); ++i) {
      const size_t prev_idx = delim_idx[i - 1];
      const size_t idx = delim_idx[i];
      result.push_back(keyexpr.substr(prev_idx + 1, idx - prev_idx - 1));
    }
  } catch (const std::exception & e) {
    printf("%s\n", e.what());
    return {};
  }
  // Finally add the last substr.
  result.push_back(keyexpr.substr(delim_idx.back() + 1));
  return result;
}

/**
 * Convert a rmw_qos_profile_t to a string with format:
 *
 * <ReliabilityKind>:<DurabilityKind>:<HistoryKind>,<HistoryDepth>"
 * Where:
 *  <ReliabilityKind> - enum value from rmw_qos_reliability_policy_e.
 *  <DurabilityKind> - enum value from rmw_qos_durability_policy_e.
 *  <HistoryKind> - enum value from rmw_qos_history_policy_e.
 *  <HistoryDepth> - The depth number.
 */
// TODO(Yadunund): Rely on maps to retrieve strings.
std::string qos_to_keyexpr(rmw_qos_profile_t qos)
{
  std::string keyexpr = "";
  keyexpr += std::to_string(qos.reliability);
  keyexpr += QOS_DELIMITER;
  keyexpr += std::to_string(qos.durability);
  keyexpr += QOS_DELIMITER;
  keyexpr += std::to_string(qos.history);
  keyexpr += QOS_HISTORY_DELIMITER;
  keyexpr += std::to_string(qos.depth);
  return keyexpr;
}

/// Convert a rmw_qos_profile_t from a keyexpr. Return std::nullopt if invalid.
std::optional<rmw_qos_profile_t> keyexpr_to_qos(const std::string & keyexpr)
{
  rmw_qos_profile_t qos;
  const std::vector<std::string> parts = split_keyexpr(keyexpr, QOS_DELIMITER);
  if (parts.size() < 3) {
    return std::nullopt;
  }
  const std::vector<std::string> history_parts = split_keyexpr(parts[2], QOS_HISTORY_DELIMITER);
  if (history_parts.size() < 2) {
    return std::nullopt;
  }
  sscanf(parts[0].c_str(), "%zu", &qos.reliability);
  sscanf(parts[1].c_str(), "%zu", &qos.durability);
  sscanf(history_parts[0].c_str(), "%zu", &qos.history);
  sscanf(history_parts[1].c_str(), "%zu", &qos.depth);

  return qos;
}

}  // namespace

///=============================================================================
std::string subscription_token(size_t domain_id)
{
  std::string token = std::string(ADMIN_SPACE) + "/" + std::to_string(domain_id) + "/**";
  return token;
}

///=============================================================================
Entity::Entity(
  std::string id,
  EntityType type,
  NodeInfo node_info,
  std::optional<TopicInfo> topic_info)
: id_(std::move(id)),
  type_(std::move(type)),
  node_info_(std::move(node_info)),
  topic_info_(std::move(topic_info))
{
  /**
   * Set the liveliness token for the particular entity.
   *
   * The liveliness token keyexprs are in the form:
   *
   * <ADMIN_SPACE>/<domainid>/<id>/<entity>/<namespace>/<nodename>
   *
   * Where:
   *  <domainid> - A number set by the user to "partition" graphs.  Roughly equivalent to the domain ID in DDS.
   *  <id> - A unique ID to identify this entity. Currently the id is the zenoh session's id with elements concatenated into a string using '.' as separator.
   *  <entity> - The type of entity.  This can be one of "NN" for a node, "MP" for a publisher, "MS" for a subscription, "SS" for a service server, or "SC" for a service client.
   *  <namespace> - The ROS namespace for this entity.  If the namespace is absolute, this function will add in an _ for later parsing reasons.
   *  <nodename> - The ROS node name for this entity.
   *
   * For entities with topic infomation, the liveliness token keyexpr have additional fields:
   *
   * <ADMIN_SPACE>/<domainid>/<id>/<entity>/<namespace>/<nodename>/<topic_name>/<topic_type>/<topic_qos>
   *  <topic_name> - The ROS topic name for this entity.
   *  <topic_type> - The type for the topic.
   *  <topic_qos> - The qos for the topic.
   */
  std::stringstream token_ss;
  const std::string & ns = node_info_.ns_;
  token_ss << ADMIN_SPACE << "/" << node_info_.domain_id_ << "/" << id_ << "/" << entity_to_str.at(
    type_) << ns;
  // An empty namespace from rcl will contain "/" but zenoh does not allow keys with "//".
  // Hence we add an "_" to denote an empty namespace such that splitting the key
  // will always result in 5 parts.
  if (ns == "/") {
    token_ss << "_/";
  } else {
    token_ss << "/";
  }
  // Finally append node name.
  token_ss << mangle_name(node_info_.name_);
  // If this entity has a topic info, append it to the token.
  if (topic_info_.has_value()) {
    const auto & topic_info = this->topic_info_.value();
    // Note: We don't append a leading "/" as we expect the ROS topic name to start with a "/".
    token_ss <<
      "/" + mangle_name(topic_info.name_) + "/" + topic_info.type_ + "/" + qos_to_keyexpr(
      topic_info.qos_);
  }

  this->keyexpr_ = token_ss.str();
}

///=============================================================================
std::optional<Entity> Entity::make(
  z_id_t id,
  EntityType type,
  NodeInfo node_info,
  std::optional<TopicInfo> topic_info)
{
  if (entity_to_str.find(type) == entity_to_str.end()) {
    RCUTILS_SET_ERROR_MSG("Invalid entity type.");
    return std::nullopt;
  }
  if (node_info.ns_.empty() || node_info.name_.empty()) {
    RCUTILS_SET_ERROR_MSG("Invalid node_info for entity.");
    return std::nullopt;
  }
  if (type != EntityType::Node && !topic_info.has_value()) {
    RCUTILS_SET_ERROR_MSG("Invalid topic_info for entity.");
    return std::nullopt;
  }

  Entity entity{zid_to_str(id), std::move(type), std::move(node_info), std::move(topic_info)};
  return entity;
}

///=============================================================================
std::optional<Entity> Entity::make(const std::string & keyexpr)
{
  std::vector<std::string> parts = split_keyexpr(keyexpr);
  // A token will contain at least 5 parts:
  // (ADMIN_SPACE, domain_id, entity_str, namespace, node_name).
  // Basic validation.
  if (parts.size() < 6) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received invalid liveliness token");
    return std::nullopt;
  }
  for (const std::string & p : parts) {
    if (p.empty()) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Received invalid liveliness token");
      return std::nullopt;
    }
  }

  if (parts[0] != ADMIN_SPACE) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received liveliness token with invalid admin space.");
    return std::nullopt;
  }

  // Get the entity, ie NN, MP, MS, SS, SC.
  std::string & entity_str = parts[3];
  std::unordered_map<std::string, EntityType>::const_iterator entity_it =
    str_to_entity.find(entity_str);
  if (entity_it == str_to_entity.end()) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received liveliness token with invalid entity %s.", entity_str.c_str());
    return std::nullopt;
  }

  EntityType entity_type = entity_it->second;
  std::size_t domain_id = std::stoul(parts[1]);
  std::string & id = parts[2];
  std::string ns = parts[4] == "_" ? "/" : "/" + std::move(parts[4]);
  std::string node_name = demangle_name(std::move(parts[5]));
  std::optional<TopicInfo> topic_info = std::nullopt;

  // Populate topic_info if we have a token for an entity other than a node.
  if (entity_type != EntityType::Node) {
    if (parts.size() < 9) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Received liveliness token for non-node entity without required parameters.");
      return std::nullopt;
    }
    std::optional<rmw_qos_profile_t> qos = keyexpr_to_qos(parts[8]);
    if (!qos.has_value()) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Received liveliness token with invalid qos keyexpr");
      return std::nullopt;
    }
    topic_info = TopicInfo{
      demangle_name(std::move(parts[6])),
      std::move(parts[7]),
      std::move(qos.value())
    };
  }

  return Entity{
    std::move(id),
    std::move(entity_type),
    NodeInfo{std::move(domain_id), std::move(ns), std::move(node_name), ""},
    std::move(topic_info)};
}

///=============================================================================
std::string Entity::id() const
{
  return this->id_;
}

///=============================================================================
EntityType Entity::type() const
{
  return this->type_;
}

std::string Entity::node_namespace() const
{
  return this->node_info_.ns_;
}

std::string Entity::node_name() const
{
  return this->node_info_.name_;
}

std::string Entity::node_enclave() const
{
  return this->node_info_.enclave_;
}

///=============================================================================
std::optional<TopicInfo> Entity::topic_info() const
{
  return this->topic_info_;
}

///=============================================================================
std::string Entity::keyexpr() const
{
  return this->keyexpr_;
}

///=============================================================================
bool PublishToken::put(
  z_owned_session_t * session,
  const std::string & token)
{
  if (!z_session_check(session)) {
    RCUTILS_SET_ERROR_MSG("The zenoh session is invalid.");
    return false;
  }

  // TODO(Yadunund): z_keyexpr_new creates a copy so find a way to avoid it.
  z_owned_keyexpr_t keyexpr = z_keyexpr_new(token.c_str());
  auto drop_keyexpr = rcpputils::make_scope_exit(
    [&keyexpr]() {
      z_drop(z_move(keyexpr));
    });
  if (!z_keyexpr_check(&keyexpr)) {
    RCUTILS_SET_ERROR_MSG("invalid keyexpression generation for liveliness publication.");
    return false;
  }
  RCUTILS_LOG_WARN_NAMED("rmw_zenoh_cpp", "Sending PUT on %s", token.c_str());
  z_put_options_t options = z_put_options_default();
  options.encoding = z_encoding(Z_ENCODING_PREFIX_EMPTY, NULL);
  if (z_put(z_loan(*session), z_keyexpr(token.c_str()), nullptr, 0, &options) < 0) {
    RCUTILS_SET_ERROR_MSG("unable to publish liveliness for node creation");
    return false;
  }

  return true;
}

///=============================================================================
bool PublishToken::del(
  z_owned_session_t * session,
  const std::string & token)
{
  if (!z_session_check(session)) {
    RCUTILS_SET_ERROR_MSG("The zenoh session is invalid.");
    return false;
  }

  // TODO(Yadunund): z_keyexpr_new creates a copy so find a way to avoid it.
  z_owned_keyexpr_t keyexpr = z_keyexpr_new(token.c_str());
  auto drop_keyexpr = rcpputils::make_scope_exit(
    [&keyexpr]() {
      z_drop(z_move(keyexpr));
    });
  if (!z_keyexpr_check(&keyexpr)) {
    RCUTILS_SET_ERROR_MSG("invalid key-expression generation for liveliness publication.");
    return false;
  }
  RCUTILS_LOG_WARN_NAMED("rmw_zenoh_cpp", "Sending DELETE on %s", token.c_str());
  const z_delete_options_t options = z_delete_options_default();
  if (z_delete(z_loan(*session), z_loan(keyexpr), &options) < 0) {
    RCUTILS_SET_ERROR_MSG("failed to delete liveliness key");
    return false;
  }

  return true;
}

///=============================================================================
std::string mangle_name(const std::string & input)
{
  std::string output = "";
  for (std::size_t i = 0; i < input.length(); ++i) {
    if (input[i] == '/') {
      output += SLASH_REPLACEMENT;
    } else {
      output += input[i];
    }
  }
  return output;
}

///=============================================================================
std::string demangle_name(const std::string & input)
{
  std::string output = "";
  for (std::size_t i = 0; i < input.length(); ++i) {
    if (input[i] == SLASH_REPLACEMENT) {
      output += '/';
    } else {
      output += input[i];
    }
  }
  return output;
}

}  // namespace liveliness
