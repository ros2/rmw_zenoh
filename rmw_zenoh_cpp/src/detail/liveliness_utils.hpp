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

#ifndef DETAIL__LIVELINESS_UTILS_HPP_
#define DETAIL__LIVELINESS_UTILS_HPP_

#include <zenoh.h>

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "rmw/types.h"

namespace liveliness
{

///=============================================================================
struct NodeInfo
{
  std::size_t domain_id_;
  std::string ns_;
  std::string name_;
  std::string enclave_;

  NodeInfo(
    std::size_t domain_id,
    std::string ns,
    std::string name,
    std::string enclave);
};

///=============================================================================
struct TopicInfo
{
  std::string name_;
  std::string type_;
  rmw_qos_profile_t qos_;

  TopicInfo(
    std::string name,
    std::string type,
    rmw_qos_profile_t qos);
};

///=============================================================================
/// Retuns the keyexpr for liveliness subscription.
std::string subscription_token(size_t domain_id);

///=============================================================================
enum class EntityType : uint8_t
{
  Invalid = 0,
  Node,
  Publisher,
  Subscription,
  Service,
  Client
};

///=============================================================================
// An struct to bundle results of parsing a token.
// TODO(Yadunund): Consider using variadic templates to pass args instead of
// relying on optional fields.
class Entity
{
public:
  // TODO(Yadunund): Find a way to better bundle the type and the associated data.
  // Also make id globally unique when zenoh supports this.
  // In the meanwhile, we rely on the interim _guid() method to return a hash value
  // of the keyexpr which we use as guid.

  /// @brief Make an Entity from datatypes. This will return nullopt if the required
  ///   fields are not present for the EntityType.
  /// @param id The zenoh
  /// @param type
  /// @param node_info
  /// @param topic_info
  /// @return
  static std::optional<Entity> make(
    z_id_t id,
    EntityType type,
    NodeInfo node_info,
    std::optional<TopicInfo> topic_info = std::nullopt);

  /// Make an Entity from a liveliness keyexpr.
  static std::optional<Entity> make(const std::string & keyexpr);

  // Get the zenoh session id as a string. This is not unique as entities
  // created within the same session, will have the same ids.
  std::string id() const;

  // Interim method to get a "unique" id for this entity which is the hash of the keyexpr.
  // Note: This is still not globally unique since two entities created with the
  // same session with the same EntityType, topic name, topic type, and topic qos
  // will have the same _id. Eg. If a node creates two publishers with same properties.
  std::size_t guid() const;

  /// Get the entity type.
  EntityType type() const;

  std::string node_namespace() const;

  std::string node_name() const;

  std::string node_enclave() const;

  /// Get the topic_info.
  std::optional<TopicInfo> topic_info() const;

  /// Get the liveliness keyexpr for this entity.
  std::string keyexpr() const;

  // Two entities are equal if their guids are equal.
  bool operator==(const Entity & other) const;

private:
  Entity(
    std::string id,
    EntityType type,
    NodeInfo node_info,
    std::optional<TopicInfo> topic_info);

  std::string id_;
  std::size_t guid_;
  EntityType type_;
  NodeInfo node_info_;
  std::optional<TopicInfo> topic_info_;
  std::string keyexpr_;
};

///=============================================================================
/// Replace "/" instances with "%".
std::string mangle_name(const std::string & input);

///=============================================================================
/// Replace "%" instances with "/".
std::string demangle_name(const std::string & input);

///=============================================================================
/**
 * Convert a rmw_qos_profile_t to a string with format:
 *
 * <ReliabilityKind>:<DurabilityKind>:<HistoryKind>,<HistoryDepth>"
 * Where:
 *  <ReliabilityKind> - enum value from rmw_qos_reliability_policy_e.
 *  <DurabilityKind> - enum value from rmw_qos_durability_policy_e.
 *  <HistoryKind> - enum value from rmw_qos_history_policy_e.
 *  <HistoryDepth> - The depth number.
 * For example, the liveliness substring for a topic with Reliability policy: reliable,
 * Durability policy: volatile, History policy: keep_last, and depth: 10, would be
 * "1:2:1,10". See rmw/types.h for the values of each policy enum.
 */
std::string qos_to_keyexpr(rmw_qos_profile_t qos);

///=============================================================================
/// Convert a rmw_qos_profile_t from a keyexpr. Return std::nullopt if invalid.
std::optional<rmw_qos_profile_t> keyexpr_to_qos(const std::string & keyexpr);

///=============================================================================
/// Convert a Zenoh id to a string.
std::string zid_to_str(const z_id_t & id);

}  // namespace liveliness

///=============================================================================
// Allow Entity to be hashed and used as a key in unordered_maps/sets
namespace std
{
template<>
struct hash<liveliness::Entity>
{
  auto operator()(const liveliness::Entity & entity) const -> size_t
  {
    return entity.guid();
  }
};
}  // namespace std

#endif  // DETAIL__LIVELINESS_UTILS_HPP_
