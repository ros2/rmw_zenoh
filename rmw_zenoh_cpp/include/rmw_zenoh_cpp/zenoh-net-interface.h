// Copyright 2020 ADLINK, Inc.
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

#ifndef RMW_ZENOH_CPP__ZENOH_NET_INTERFACE_H_
#define RMW_ZENOH_CPP__ZENOH_NET_INTERFACE_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>


/**
 * The kind of consolidation that should be applied on replies to a :c:func:`zn_query`.
 *
 *     - **zn_consolidation_mode_t_FULL**: Guaranties unicity of replies. Optimizes bandwidth.
 *     - **zn_consolidation_mode_t_LAZY**: Does not garanty unicity. Optimizes latency.
 *     - **zn_consolidation_mode_t_NONE**: No consolidation.
 */
typedef enum zn_consolidation_mode_t
{
  zn_consolidation_mode_t_FULL,
  zn_consolidation_mode_t_LAZY,
  zn_consolidation_mode_t_NONE,
} zn_consolidation_mode_t;

/**
 * The subscription reliability.
 *
 *     - **zn_reliability_t_BEST_EFFORT**
 *     - **zn_reliability_t_RELIABLE**
 */
typedef enum zn_reliability_t
{
  zn_reliability_t_BEST_EFFORT,
  zn_reliability_t_RELIABLE,
} zn_reliability_t;

/**
 * The subscription mode.
 *
 *     - **zn_submode_t_PUSH**
 *     - **zn_submode_t_PULL**
 */
typedef enum zn_submode_t
{
  zn_submode_t_PUSH,
  zn_submode_t_PULL,
} zn_submode_t;

typedef struct zn_properties_t zn_properties_t;

typedef struct zn_publisher_t zn_publisher_t;

typedef struct zn_query_t zn_query_t;

typedef struct zn_queryable_t zn_queryable_t;

typedef struct zn_session_t zn_session_t;

typedef struct zn_subscriber_t zn_subscriber_t;

/**
 * A string.
 *
 * Members:
 *   const char *val: A pointer to the string.
 *   unsigned int len: The length of the string.
 *
 */
typedef struct z_string_t
{
  const char * val;
  size_t len;
} z_string_t;

/**
 * A resource key.
 *
 * Resources are identified by URI like string names.
 * Examples : ``"/some/resource/key"``.
 * Resource names can be mapped to numerical ids through :c:func:`zn_declare_resource`
 * for wire and computation efficiency.
 *
 * A resource key can be either:
 *
 *   - A plain string resource name.
 *   - A pure numerical id.
 *   - The combination of a numerical prefix and a string suffix.
 *
 * Members:
 *   unsigned long id: The id or prefix of this resource key. ``0`` if empty.
 *   const char *suffix: The suffix of this resource key. ``NULL`` if pure numerical id.
 */
typedef struct zn_reskey_t
{
  unsigned long id;
  const char * suffix;
} zn_reskey_t;

/**
 * The subscription period.
 *
 * Members:
 *     unsigned int origin:
 *     unsigned int period:
 *     unsigned int duration:
 */
typedef struct zn_period_t
{
  unsigned int origin;
  unsigned int period;
  unsigned int duration;
} zn_period_t;

/**
 * Informations to be passed to :c:func:`zn_declare_subscriber` to configure the created :c:type:`zn_subscriber_t`.
 *
 * Members:
 *     zn_reliability_t reliability: The subscription reliability.
 *     zn_submode_t mode: The subscription mode.
 *     zn_period_t *period: The subscription period.
 */
typedef struct zn_subinfo_t
{
  zn_reliability_t reliability;
  zn_submode_t mode;
  zn_period_t * period;
} zn_subinfo_t;

/**
 * An array of bytes.
 *
 * Members:
 *   const unsigned char *val: A pointer to the bytes array.
 *   unsigned int len: The length of the bytes array.
 *
 */
typedef struct z_bytes_t
{
  const uint8_t * val;
  size_t len;
} z_bytes_t;

/**
 * A zenoh-net data sample.
 *
 * A sample is the value associated to a given resource at a given point in time.
 *
 * Members:
 *   z_string_t key: The resource key of this data sample.
 *   z_bytes_t value: The value of this data sample.
 */
typedef struct zn_sample_t
{
  z_string_t key;
  z_bytes_t value;
} zn_sample_t;

/**
 * An array of NULL terminated strings.
 *
 * Members:
 *   char *const *val: A pointer to the array.
 *   unsigned int len: The length of the array.
 *
 */
typedef struct z_str_array_t
{
  const char * const * val;
  size_t len;
} z_str_array_t;

/**
 * A hello message returned by a zenoh entity to a scout message sent with :c:func:`zn_scout`.
 *
 * Members:
 *   unsigned int whatami: The kind of zenoh entity.
 *   z_bytes_t pid: The peer id of the scouted entity (empty if absent).
 *   z_str_array_t locators: The locators of the scouted entity.
 *
 */
typedef struct zn_hello_t
{
  unsigned int whatami;
  z_bytes_t pid;
  z_str_array_t locators;
} zn_hello_t;

/**
 * An array of :c:struct:`zn_hello_t` messages.
 *
 * Members:
 *   const zn_hello_t *val: A pointer to the array.
 *   unsigned int len: The length of the array.
 *
 */
typedef struct zn_hello_array_t
{
  const zn_hello_t * val;
  size_t len;
} zn_hello_array_t;

/**
 * The possible values of :c:member:`zn_target_t.tag`.
 *
 *     - **zn_target_t_BEST_MATCHING**: The nearest complete queryable if any else all matching queryables.
 *     - **zn_target_t_COMPLETE**: A set of complete queryables.
 *     - **zn_target_t_ALL**: All matching queryables.
 *     - **zn_target_t_NONE**: No queryables.
 */
typedef enum zn_target_t_Tag
{
  zn_target_t_BEST_MATCHING,
  zn_target_t_COMPLETE,
  zn_target_t_ALL,
  zn_target_t_NONE,
} zn_target_t_Tag;

typedef struct zn_target_t_COMPLETE_Body
{
  unsigned int n;
} zn_target_t_COMPLETE_Body;

typedef struct zn_target_t
{
  zn_target_t_Tag tag;
  union {
    zn_target_t_COMPLETE_Body complete;
  };
} zn_target_t;

/**
 * The zenoh-net queryables that should be target of a :c:func:`zn_query`.
 *
 * Members:
 *     unsigned int kind: A mask of queryable kinds.
 *     zn_target_t target: The query target.
 */
typedef struct zn_query_target_t
{
  unsigned int kind;
  zn_target_t target;
} zn_query_target_t;

/**
 * The kind of consolidation that should be applied on replies to a :c:func:`zn_query`
 * at the different stages of the reply process.
 *
 * Members:
 *   zn_consolidation_mode_t first_routers: The consolidation mode to apply on first routers of the replies routing path.
 *   zn_consolidation_mode_t last_router: The consolidation mode to apply on last router of the replies routing path.
 *   zn_consolidation_mode_t reception: The consolidation mode to apply at reception of the replies.
 */
typedef struct zn_query_consolidation_t
{
  zn_consolidation_mode_t first_routers;
  zn_consolidation_mode_t last_router;
  zn_consolidation_mode_t reception;
} zn_query_consolidation_t;

/**
 * Information on the source of a reply.
 *
 * Members:
 *   unsigned int kind: The kind of source.
 *   z_bytes_t id: The unique id of the source.
 */
typedef struct zn_source_info_t
{
  unsigned int kind;
  z_bytes_t id;
} zn_source_info_t;

extern const unsigned int ZN_ROUTER;

extern const unsigned int ZN_PEER;

extern const unsigned int ZN_CLIENT;

extern const unsigned int ZN_QUERYABLE_ALL_KINDS;

// NOTE(esteve): defined as a define in zenoh-pico, declared as a global static variable in zenoh-c
#define ZN_QUERYABLE_STORAGE 0x02   // 1 << 1

extern const unsigned int ZN_QUERYABLE_EVAL;

extern const unsigned int ZN_CONFIG_MODE_KEY;

extern const unsigned int ZN_CONFIG_PEER_KEY;

extern const unsigned int ZN_CONFIG_LISTENER_KEY;

extern const unsigned int ZN_CONFIG_USER_KEY;

extern const unsigned int ZN_CONFIG_PASSWORD_KEY;

extern const unsigned int ZN_CONFIG_MULTICAST_SCOUTING_KEY;

extern const unsigned int ZN_CONFIG_MULTICAST_INTERFACE_KEY;

extern const unsigned int ZN_CONFIG_MULTICAST_ADDRESS_KEY;

extern const unsigned int ZN_CONFIG_SCOUTING_TIMEOUT_KEY;

extern const unsigned int ZN_CONFIG_SCOUTING_DELAY_KEY;

extern const unsigned int ZN_CONFIG_ADD_TIMESTAMP_KEY;

extern const unsigned int ZN_CONFIG_LOCAL_ROUTING_KEY;

extern const unsigned int ZN_INFO_PID_KEY;

extern const unsigned int ZN_INFO_PEER_PID_KEY;

extern const unsigned int ZN_INFO_ROUTER_PID_KEY;

/**
 * Initialise the zenoh runtime logger
 *
 */
void z_init_logger(void);

/**
 * Construct a :c:type:`z_string_t` from a NULL terminated string.
 * The content of the given string is copied.
 *
 * Parameters:
 *     s: The NULL terminated string.
 *
 * Returns:
 *     A new :c:type:`z_string_t`.
 */
z_string_t z_string_make(const char * s);

/**
 * Close a zenoh-net session.
 *
 * Parameters:
 *     session: A zenoh-net session.
 */
void zn_close(zn_session_t * session);

/**
 * Notifies the zenoh runtime that there won't be any more replies sent for this
 * query.
 */
void zn_close_query(zn_query_t * query);

/**
 * Create a default set of properties for client mode zenoh-net session configuration.
 * If peer is not null, it is added to the configuration as remote peer.
 *
 * Parameters:
 *   peer: An optional peer locator.
 */
zn_properties_t * zn_config_client(char * peer);

/**
 * Create a default set of properties for zenoh-net session configuration.
 */
zn_properties_t * zn_config_default(void);

/**
 * Create an empty set of properties for zenoh-net session configuration.
 */
zn_properties_t * zn_config_empty(void);

/**
 * Create a default set of properties for peer mode zenoh-net session configuration.
 */
zn_properties_t * zn_config_peer(void);

/**
 * Declare a :c:type:`zn_publisher_t` for the given resource key.
 *
 * Written resources that match the given key will only be sent on the network
 * if matching subscribers exist in the system.
 *
 * Parameters:
 *     session: The zenoh-net session.
 *     resource: The resource key to publish.
 *
 * Returns:
 *    The created :c:type:`zn_publisher_t` or null if the declaration failed.
 */
zn_publisher_t * zn_declare_publisher(zn_session_t * session, zn_reskey_t reskey);

/**
 * Declare a :c:type:`zn_queryable_t` for the given resource key.
 *
 * Parameters:
 *     session: The zenoh-net session.
 *     resource: The resource key the :c:type:`zn_queryable_t` will reply to.
 *     kind: The kind of :c:type:`zn_queryable_t`.
 *     callback: The callback function that will be called each time a matching query is received.
 *     arg: A pointer that will be passed to the **callback** on each call.
 *
 * Returns:
 *    The created :c:type:`zn_queryable_t` or null if the declaration failed.
 */
zn_queryable_t * zn_declare_queryable(
  zn_session_t * session,
  zn_reskey_t reskey,
  unsigned int kind,
  void (* callback)(zn_query_t *, const void *),
  void * arg);

/**
 * Associate a numerical id with the given resource key.
 *
 * This numerical id will be used on the network to save bandwidth and
 * ease the retrieval of the concerned resource in the routing tables.
 *
 * Parameters:
 *     session: The zenoh-net session.
 *     resource: The resource key to map to a numerical id.
 *
 * Returns:
 *     A numerical id.
 */
unsigned long zn_declare_resource(zn_session_t * session, zn_reskey_t reskey);

/**
 * Declare a :c:type:`zn_subscriber_t` for the given resource key.
 *
 * Parameters:
 *     session: The zenoh-net session.
 *     resource: The resource key to subscribe.
 *     sub_info: The :c:type:`zn_subinfo_t` to configure the :c:type:`zn_subscriber_t`.
 *     callback: The callback function that will be called each time a data matching the subscribed resource is received.
 *     arg: A pointer that will be passed to the **callback** on each call.
 *
 * Returns:
 *    The created :c:type:`zn_subscriber_t` or null if the declaration failed.
 */
zn_subscriber_t * zn_declare_subscriber(
  zn_session_t * session,
  zn_reskey_t reskey,
  zn_subinfo_t sub_info,
  void (* callback)(const zn_sample_t *, const void *),
  void * arg);

/**
 * Free an array of :c:struct:`zn_hello_t` messages and it's contained :c:struct:`zn_hello_t` messages recursively.
 *
 * Parameters:
 *     strs: The array of :c:struct:`zn_hello_t` messages to free.
 *
 */
void zn_hello_array_free(zn_hello_array_t hellos);

/**
 * Get informations about an zenoh-net session.
 *
 * Parameters:
 *     session: A zenoh-net session.
 *
 * Returns:
 *     A :c:type:`zn_properties_t` map containing informations on the given zenoh-net session.
 */
zn_properties_t * zn_info(zn_session_t * session);

/**
 * Open a zenoh-net session
 *
 * Parameters:
 *     config: A set of properties.
 *
 * Returns:
 *     The created zenoh-net session or null if the creation did not succeed.
 */
zn_session_t * zn_open(zn_properties_t * config);

/**
 * Free a set of properties.
 *
 * Parameters:
 *   ps: A pointer to the properties.
 */
void zn_properties_free(zn_properties_t * ps);

/**
 * Get the property with the given key from a properties map.
 *
 * Parameters:
 *     ps: A pointer to properties map.
 *     key: The key of the property.
 *
 * Returns:
 *     The value of the property with key ``key`` in properties map ``ps``.
 */
z_string_t zn_properties_get(zn_properties_t * ps, unsigned int key);

/**
 * Insert a property with a given key to a properties map.
 * If a property with the same key already exists in the properties map, it is replaced.
 *
 * Parameters:
 *   ps: A pointer to the properties map.
 *   key: The key of the property to add.
 *   value: The value of the property to add.
 *
 * Returns:
 *     A pointer to the updated properties map.
 */
zn_properties_t * zn_properties_insert(zn_properties_t * ps, unsigned long key, z_string_t value);

/**
 * Get the length of the given properties map.
 *
 * Parameters:
 *     ps: A pointer to the properties map.
 *
 * Returns:
 *     The length of the given properties map.
 */
unsigned int zn_properties_len(zn_properties_t * ps);

/**
 * Return a new empty map of properties.
 */
zn_properties_t * zn_properties_make(void);

/**
 * Pull data for a pull mode :c:type:`zn_subscriber_t`. The pulled data will be provided
 * by calling the **callback** function provided to the :c:func:`zn_declare_subscriber` function.
 *
 * Parameters:
 *     sub: The :c:type:`zn_subscriber_t` to pull from.
 */
void zn_pull(zn_subscriber_t * sub);

/**
 * Query data from the matching queryables in the system.
 *
 * Parameters:
 *     session: The zenoh-net session.
 *     resource: The resource key to query.
 *     predicate: An indication to matching queryables about the queried data.
 *     target: The kind of queryables that should be target of this query.
 *     consolidation: The kind of consolidation that should be applied on replies.
 *     callback: The callback function that will be called on reception of replies for this query.
 *     arg: A pointer that will be passed to the **callback** on each call.
 */
void zn_query(
  zn_session_t * session,
  zn_reskey_t reskey,
  const char * predicate,
  zn_query_target_t target,
  zn_query_consolidation_t consolidation,
  void (* callback)(const zn_source_info_t *, const zn_sample_t *, const void *),
  void * arg);

/**
 * Create a default :c:type:`zn_query_consolidation_t`.
 */
zn_query_consolidation_t zn_query_consolidation_default(void);

/**
 * Return the predicate for this query
 */
z_string_t zn_query_predicate(zn_query_t * query);

/**
 * Return the resource name for this query
 */
z_string_t zn_query_res_name(zn_query_t * query);

/**
 * Create a default :c:type:`zn_query_target_t`.
 */
zn_query_target_t zn_query_target_default(void);

/**
 * Create a resource key from a resource id.
 *
 * Parameters:
 *     id: The resource id.
 *
 * Returns:
 *     A new resource key.
 */
zn_reskey_t zn_rid(unsigned long id);

/**
 * Create a resource key from a resource id and a suffix.
 *
 * Parameters:
 *     id: The resource id.
 *     suffix: The suffix.
 *
 * Returns:
 *     A new resource key.
 */
zn_reskey_t zn_rid_with_suffix(unsigned long id, const char * suffix);

/**
 * Create a resource key from a resource name.
 *
 * Parameters:
 *     id: The resource name.
 *
 * Returns:
 *     A new resource key.
 */
zn_reskey_t zn_rname(const char * name);

/**
 * Scout for routers and/or peers.
 *
 * Parameters:
 *     what: A whatami bitmask of zenoh entities kind to scout for.
 *     config: A set of properties to configure the scouting.
 *     scout_period: The time that should be spent scouting before returnng the results.
 *
 * Returns:
 *     An array of :c:struct:`zn_hello_t` messages.
 */
zn_hello_array_t zn_scout(unsigned int what, zn_properties_t * config, unsigned long scout_period);

/**
 * Sends a reply to a query.
 */
void zn_send_reply(
  zn_query_t * query,
  const char * key,
  const unsigned char * payload,
  unsigned int len);

/**
 * Free an array of NULL terminated strings and it's contained NULL terminated strings recursively.
 *
 * Parameters:
 *     strs: The array of NULL terminated strings to free.
 *
 */
void zn_str_array_free(z_str_array_t strs);

/**
 * Create a default subscription info.
 */
zn_subinfo_t zn_subinfo_default(void);

/**
 * Create a default :c:type:`zn_target_t`.
 */
zn_target_t zn_target_default(void);

/**
 * Undeclare a :c:type:`zn_publisher_t`.
 *
 * Parameters:
 *     sub: The :c:type:`zn_publisher_t` to undeclare.
 */
void zn_undeclare_publisher(zn_publisher_t * publ);

/**
 * Undeclare a :c:type:`zn_queryable_t`.
 *
 * Parameters:
 *     qable: The :c:type:`zn_queryable_t` to undeclare.
 */
void zn_undeclare_queryable(zn_queryable_t * qable);

/**
 * Undeclare a :c:type:`zn_subscriber_t`.
 *
 * Parameters:
 *     sub: The :c:type:`zn_subscriber_t` to undeclare.
 */
void zn_undeclare_subscriber(zn_subscriber_t * sub);

/**
 * Write data.
 *
 * Parameters:
 *     session: The zenoh-net session.
 *     resource: The resource key to write.
 *     payload: The value to write.
 *     len: The length of the value to write.
 * Returns:
 *     ``0`` in case of success, ``1`` in case of failure.
 */
int zn_write(zn_session_t * session, zn_reskey_t reskey, const char * payload, unsigned int len);

/*------------------ Zenoh-pico operations ------------------*/
/**
 * Read from the network. This function should be called manually called when
 * the read loop has not been started, e.g., when running in a single thread.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_read(zn_session_t * z);

/**
 * Start a separate task to read from the network and process the messages
 * as soon as they are received. Note that the task can be implemented in
 * form of thread, process, etc. and its implementation is platform-dependent.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_start_read_task(zn_session_t * z);

/**
 * Stop the read task. This may result in stopping a thread or a process depending
 * on the target platform.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_stop_read_task(zn_session_t * z);

/**
 * Send a KeepAlive message.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_send_keep_alive(zn_session_t * z);

/**
 * Start a separate task to handle the session lease. This task will send ``KeepAlive``
 * messages when needed and will close the session when the lease is expired. Note that
 * the task can be implemented in form of thread, process, etc. and its implementation
 * is platform-dependent.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_start_lease_task(zn_session_t * z);

/**
 * Stop the lease task. This may result in stopping a thread or a process depending
 * on the target platform.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_stop_lease_task(zn_session_t * z);

#endif  // RMW_ZENOH_CPP__ZENOH_NET_INTERFACE_H_
