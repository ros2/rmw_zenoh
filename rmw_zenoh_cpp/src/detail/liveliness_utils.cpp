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

#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcpputils/scope_exit.hpp"
#include "rcutils/logging_macros.h"

#include "rmw/error_handling.h"


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
/// Enum of liveliness key-expression components.
enum KeyexprIndex
{
  AdminSpace,
  DomainId,
  Zid,
  Id,
  EntityStr,
  Namespace,
  NodeName,
  TopicName,
  TopicType,
  TopicQoS
};

// Every keyexpression will have components upto node name.
#define KEYEXPR_INDEX_MIN KeyexprIndex::NodeName
#define KEYEXPR_INDEX_MAX KeyexprIndex::TopicQoS

/// The admin space used to prefix the liveliness tokens.
static const char ADMIN_SPACE[] = "@ros2_lv";
static const char NODE_STR[] = "NN";
static const char PUB_STR[] = "MP";
static const char SUB_STR[] = "MS";
static const char SRV_STR[] = "SS";
static const char CLI_STR[] = "SC";
static const char EMPTY_NAMESPACE_REPLACEMENT = '_';
static const char KEYEXPR_DELIMITER = '/';
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

static const std::unordered_map<std::string, rmw_qos_history_policy_e> str_to_qos_history = {
  {std::to_string(RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT), RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT},
  {std::to_string(RMW_QOS_POLICY_HISTORY_KEEP_LAST), RMW_QOS_POLICY_HISTORY_KEEP_LAST},
  {std::to_string(RMW_QOS_POLICY_HISTORY_KEEP_ALL), RMW_QOS_POLICY_HISTORY_KEEP_ALL},
  {std::to_string(RMW_QOS_POLICY_HISTORY_UNKNOWN), RMW_QOS_POLICY_HISTORY_UNKNOWN}
};

static const std::unordered_map<std::string,
  rmw_qos_reliability_policy_e> str_to_qos_reliability = {
  {std::to_string(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT),
    RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT},
  {std::to_string(RMW_QOS_POLICY_RELIABILITY_RELIABLE), RMW_QOS_POLICY_RELIABILITY_RELIABLE},
  {std::to_string(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT), RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT},
  {std::to_string(RMW_QOS_POLICY_RELIABILITY_UNKNOWN), RMW_QOS_POLICY_RELIABILITY_UNKNOWN}
};

static const std::unordered_map<std::string, rmw_qos_durability_policy_e> str_to_qos_durability = {
  {std::to_string(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT),
    RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT},
  {std::to_string(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
  {std::to_string(RMW_QOS_POLICY_DURABILITY_VOLATILE), RMW_QOS_POLICY_DURABILITY_VOLATILE},
  {std::to_string(RMW_QOS_POLICY_DURABILITY_UNKNOWN), RMW_QOS_POLICY_DURABILITY_UNKNOWN}
};

std::vector<std::string> split_keyexpr(
  const std::string & keyexpr,
  const char delim = KEYEXPR_DELIMITER)
{
  std::vector<std::string> result = {};
  size_t start = 0;
  size_t end = keyexpr.find(delim);
  while (end != std::string::npos) {
    result.push_back(keyexpr.substr(start, end - start));
    start = end + 1;
    end = keyexpr.find(delim, start);
  }
  // Finally add the last substr.
  result.push_back(keyexpr.substr(start));
  return result;
}
}  // namespace

///=============================================================================
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

///=============================================================================
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

  try {
    qos.history = str_to_qos_history.at(history_parts[0]);
    qos.reliability = str_to_qos_reliability.at(parts[0]);
    qos.durability = str_to_qos_durability.at(parts[1]);
  } catch (const std::exception & e) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("Error setting QoS values from strings: %s", e.what());
    return std::nullopt;
  }
  // Get the history depth.
  errno = 0;
  char * endptr;
  size_t num = strtoul(history_parts[1].c_str(), &endptr, 10);
  if (endptr == history_parts[1].c_str()) {
    // No values were converted, this is an error
    RMW_SET_ERROR_MSG("no valid numbers available");
    return std::nullopt;
  } else if (*endptr != '\0') {
    // There was junk after the number
    RMW_SET_ERROR_MSG("non-numeric values");
    return std::nullopt;
  } else if (errno != 0) {
    // Some other error occurred, which may include overflow or underflow
    RMW_SET_ERROR_MSG(
      "an undefined error occurred while getting the number, this may be an overflow");
    return std::nullopt;
  }
  qos.depth = num;

  // Liveliness is always automatic given liveliness tokens.
  qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  qos.liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;

  // TODO(Yadunund): Update once we support these settings.
  qos.deadline = RMW_QOS_DEADLINE_DEFAULT;
  qos.lifespan = RMW_QOS_LIFESPAN_DEFAULT;
  return qos;
}

///=============================================================================
std::string zid_to_str(const z_id_t & id)
{
  std::stringstream ss;
  ss << std::hex;
  size_t i = 0;
  for (; i < (sizeof(id.id)); i++) {
    ss << static_cast<int>(id.id[i]);
  }
  return ss.str();
}

///=============================================================================
std::string subscription_token(size_t domain_id)
{
  std::string token = std::string(ADMIN_SPACE) + "/" + std::to_string(domain_id) + "/**";
  return token;
}

///=============================================================================
Entity::Entity(
  std::string zid,
  std::string id,
  EntityType type,
  NodeInfo node_info,
  std::optional<TopicInfo> topic_info)
: zid_(std::move(zid)),
  id_(std::move(id)),
  type_(std::move(type)),
  node_info_(std::move(node_info)),
  topic_info_(std::move(topic_info))
{
  std::string keyexpr_parts[KEYEXPR_INDEX_MAX + 1] {};
  keyexpr_parts[KeyexprIndex::AdminSpace] = ADMIN_SPACE;
  keyexpr_parts[KeyexprIndex::DomainId] = std::to_string(node_info_.domain_id_);
  keyexpr_parts[KeyexprIndex::Zid] = zid_;
  keyexpr_parts[KeyexprIndex::Id] = id_;
  keyexpr_parts[KeyexprIndex::EntityStr] = entity_to_str.at(type_);
  // An empty namespace from rcl will contain "/" but zenoh does not allow keys with "//".
  // Hence we add an "_" to denote an empty namespace such that splitting the key
  // will always result in 5 parts.
  keyexpr_parts[KeyexprIndex::Namespace] = mangle_name(node_info_.ns_);
  keyexpr_parts[KeyexprIndex::NodeName] = mangle_name(node_info_.name_);
  // If this entity has a topic info, append it to the token.
  if (topic_info_.has_value()) {
    const auto & topic_info = this->topic_info_.value();
    keyexpr_parts[KeyexprIndex::TopicName] = mangle_name(topic_info.name_);
    keyexpr_parts[KeyexprIndex::TopicType] = mangle_name(topic_info.type_);
    keyexpr_parts[KeyexprIndex::TopicQoS] = qos_to_keyexpr(topic_info.qos_);
  }

  for (std::size_t i = 0; i < KEYEXPR_INDEX_MAX + 1; ++i) {
    bool last = false;
    if (!keyexpr_parts[i].empty()) {
      this->keyexpr_ += std::move(keyexpr_parts[i]);
    }
    if (i == KEYEXPR_INDEX_MAX || keyexpr_parts[i + 1].empty()) {
      last = true;
    }
    if (last) {
      break;
    }
    // Append the delimiter unless it is the last component.
    this->keyexpr_ += KEYEXPR_DELIMITER;
  }
  this->guid_ = std::hash<std::string>{}(this->keyexpr_);
}

///=============================================================================
std::optional<Entity> Entity::make(
  z_id_t zid,
  const std::string & id,
  EntityType type,
  NodeInfo node_info,
  std::optional<TopicInfo> topic_info)
{
  if (id.empty()) {
    RCUTILS_SET_ERROR_MSG("Invalid id.");
    return std::nullopt;
  }
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

  Entity entity{zid_to_str(zid), std::move(id), std::move(type), std::move(node_info), std::move(
      topic_info)};
  return entity;
}

///=============================================================================
std::optional<Entity> Entity::make(const std::string & keyexpr)
{
  std::vector<std::string> parts = split_keyexpr(keyexpr);
  // Every token will contain at least 7 parts:
  // (ADMIN_SPACE, domain_id, zid, id, entity_type, namespace, node_name).
  // Basic validation.
  if (parts.size() < KEYEXPR_INDEX_MIN + 1) {
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

  if (parts[KeyexprIndex::AdminSpace] != ADMIN_SPACE) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received liveliness token with invalid admin space.");
    return std::nullopt;
  }

  // Get the entity, ie NN, MP, MS, SS, SC.
  std::string & entity_str = parts[KeyexprIndex::EntityStr];
  std::unordered_map<std::string, EntityType>::const_iterator entity_it =
    str_to_entity.find(entity_str);
  if (entity_it == str_to_entity.end()) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received liveliness token with invalid entity %s.", entity_str.c_str());
    return std::nullopt;
  }

  EntityType entity_type = entity_it->second;
  std::size_t domain_id = std::stoul(parts[KeyexprIndex::DomainId]);
  std::string & zid = parts[KeyexprIndex::Zid];
  std::string & id = parts[KeyexprIndex::Id];
  std::string ns = demangle_name(std::move(parts[KeyexprIndex::Namespace]));
  std::string node_name = demangle_name(std::move(parts[KeyexprIndex::NodeName]));
  std::optional<TopicInfo> topic_info = std::nullopt;

  // Populate topic_info if we have a token for an entity other than a node.
  if (entity_type != EntityType::Node) {
    if (parts.size() < KEYEXPR_INDEX_MAX + 1) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Received liveliness token for non-node entity without required parameters.");
      return std::nullopt;
    }
    std::optional<rmw_qos_profile_t> qos = keyexpr_to_qos(parts[KeyexprIndex::TopicQoS]);
    if (!qos.has_value()) {
      RCUTILS_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Received liveliness token with invalid qos keyexpr");
      return std::nullopt;
    }
    topic_info = TopicInfo{
      demangle_name(std::move(parts[KeyexprIndex::TopicName])),
      demangle_name(std::move(parts[KeyexprIndex::TopicType])),
      std::move(qos.value())
    };
  }

  return Entity{
    std::move(zid),
    std::move(id),
    std::move(entity_type),
    NodeInfo{std::move(domain_id), std::move(ns), std::move(node_name), ""},
    std::move(topic_info)};
}

///=============================================================================
std::string Entity::zid() const
{
  return this->zid_;
}

///=============================================================================
std::string Entity::id() const
{
  return this->id_;
}

///=============================================================================
std::size_t Entity::guid() const
{
  return this->guid_;
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
bool Entity::operator==(const Entity & other) const
{
  // TODO(Yadunund): If we decide to directly store the guid as a
  // rmw_gid_t type, we should rely on rmw_compare_gids_equal() instead.
  return other.guid() == guid_;
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
