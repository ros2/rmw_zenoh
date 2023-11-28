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

#include <optional>
#include <string>
#include <vector>


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
  std::string qos_;

  TopicInfo(
    std::string name,
    std::string type,
    std::string qos);
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
  /// Make an Entity from datatypes. This will return nullopt if the required
  /// fields are not present for the EntityType.
  // TODO(Yadunund): Find a way to better bundle the type and the associated data.
  static std::optional<Entity> make(
    EntityType type,
    NodeInfo node_info,
    std::optional<TopicInfo> topic_info = std::nullopt);

  /// Make an Entity from a liveliness keyexpr.
  static std::optional<Entity> make(const std::string & keyexpr);

  /// Get the entity type.
  EntityType type() const;

  std::string node_namespace() const;

  std::string node_name() const;

  std::string node_enclave() const;

  /// Get the topic_info.
  std::optional<TopicInfo> topic_info() const;

  /// Get the liveliness keyexpr for this entity.
  std::string keyexpr() const;

private:
  Entity(
    EntityType type,
    NodeInfo node_info,
    std::optional<TopicInfo> topic_info);

  EntityType type_;
  NodeInfo node_info_;
  std::optional<TopicInfo> topic_info_;
  std::string keyexpr_;
};

///=============================================================================
/// Helper utilities to put/delete tokens until liveliness is supported in the
/// zenoh-c bindings.
class PublishToken
{
public:
  static bool put(
    z_owned_session_t * session,
    const std::string & token);

  static bool del(
    z_owned_session_t * session,
    const std::string & token);
};

}  // namespace liveliness

#endif  // DETAIL__LIVELINESS_UTILS_HPP_
