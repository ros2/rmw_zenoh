# Design

## Introduction

`rmw_zenoh_cpp` maps the ROS 2 [RMW API](https://github.com/ros2/rmw/tree/rolling/rmw/include/rmw) as of late 2023 onto Zenoh APIs, using [zenoh-c](https://github.com/eclipse-zenoh/zenoh-c).
The end result is that users can use ROS 2 to send and receive data over Zenoh, using the APIs that they are already familiar with.

## Brief overview

There is more detail on each item below, but a brief overview on how this is accomplished is the following:

* It is assumed that a Zenoh router is running on the local system.  This router will be used for discovery and host-to-host communication.  However it is *not* used for intra-host comms (i.e., as a message broker); that is done via direct peer-to-peer connections.
* Each "context" in ROS 2 is mapped to a single Zenoh "session".  That means that there may be many publishers, subscriptions, services, and clients sharing the same session.
* Every "context" has a local "graph cache" that keeps track of the details of the network graph of ROS 2 entities.
* Zenoh publishers, subscriptions, services, and clients are created or destroyed when the corresponding RMW APIs are called.
* Data is sent and received through the appropriate zenoh-c API when the corresponding RMW APIs are called.

The following diagram shows the default network topology of a subsystem composed of 3 nodes:

```mermaid
flowchart TB
    %% COLORS %%
    classDef blue fill:#2374f7,stroke:#000,stroke-width:2px,color:#fff
    classDef red fill:#ed2633,stroke:#000,stroke-width:2px,color:#fff
    classDef green fill:#16b522,stroke:#000,stroke-width:2px,color:#fff
    classDef orange fill:#fc822b,stroke:#000,stroke-width:2px,color:#fff
    classDef purple fill:#a724f7,stroke:#000,stroke-width:2px,color:#fff
    classDef yellow fill:#a724f7,stroke:#000,stroke-width:2px,color:#fff

    %% DIAGRAM %%
    Router(Zenoh Router:\n tcp/localhost:7447):::yellow

    %% Discovery connections %%

    Router <-..-> |discovery| S1(["Zenoh Session\n(Pub)"]):::blue
    Router <-..-> |discovery| S2(["Zenoh Session\n(Sub)"]):::blue
    Router <-..-> |discovery| S3(["Zenoh Session\n(Sub)"]):::blue

    subgraph Sessions

      %% P2P connections %%
      S1 <--> |p2p| S2
      S1 <--> |p2p| S3
      S2 <--> |p2p| S3

      linkStyle 3 stroke:red
      linkStyle 4 stroke:red
      linkStyle 5 stroke:red

      %% Data connections %%
      S1 --> |Data| S2
      S1 --> |Data| S3

      linkStyle 6 stroke:green
      linkStyle 7 stroke:green
    end
```
Default Configuration for Zenoh Sessions:
| Config | Zenoh Session    | Zenoh Router    |
| :---:   | :---: | :---: |
| Mode | Peer   | Router   |
| Connect | tcp/localhost:7447   |  -  |
| UDP Multicast | Disabled | Disabled   |
| Gossip Scouting | Enabled | Enabled   |

## Router

Zenoh has the ability to do discovery using local multicast announcements.
However, local multicast has some limitations, both intrinsic and specific to Zenoh:

* Multicast discovery can cause a lot of discovery traffic while discovering all other entities in the graph.
* Multicast discovery has a limited TTL (time-to-live), which means it can usually only discover peers on the local network segment.

To alleviate issues with multicast discovery, `rmw_zenoh_cpp` relies on a Zenoh router to discover peers and forward this discovery information to other peers via Zenoh's `gossip scouting` functionality. Hence `rmw_zenoh_cpp` requires the Zenoh router to be running.

It should be noted that when building upstream Zenoh from source, a `zenohd` binary is created which is the router.
`rmw_zenoh_cpp` actually has its own simplified version of the router that nonetheless uses most of the same code.
This was done so that Zenoh didn't have to be vendored twice (once for zenoh-c and once for zenohd), and so that the router could be more easily integrated into the ROS 2 package format.

As of 2024-02-09, the user is expected to launch the router by hand.
In the future, additional work may be done to automatically launch a Zenoh router if one isn't already running.

## Serialization/Deserialization

By default, Zenoh doesn't specify a serialization format; it just delivers bags of bytes.
There are quite a few serialization formats that `rmw_zenoh_cpp` could use, including protobuf, CDR, Cap'n Proto, JSON, etc.
In the current design, `rmw_zenoh_cpp` uses CDR as the serialization format for the following reasons:

* Using CDR means that data can be bridged between Zenoh and DDS without a deserialization/serialization step.
* Using CDR means that a new typesupport doesn't have to be developed, which is its own project.

### Related RMW APIs

* rmw_get_serialized_message_size
* rmw_serialize
* rmw_deserialize

### Related Zenoh-c APIs

N/A

## Graph Cache

One of the biggest impedance mismatches between Zenoh and ROS 2 has to do with graph introspection.
Zenoh attempts to do the absolute minimum of discovery (for performance reasons), while ROS 2 generally assumes that the entire graph is available from all entities in the system (for debugging/introspection reasons).
To deal with this discrepancy, each context in `rmw_zenoh_cpp` keeps a cache of all entities discovered in the graph so far.
An "entity" is a node, publisher, subscription, service server, or service client.
Each entity sends a unique liveliness token as it comes online, and removes that liveliness token when it is destroyed.
The key expression of these liveliness tokens encode information about the entity and it's relationship to the other entities in the system (for instance, a publisher is always attached to a node within a certain namespace).

### Related RMW APIs

* rmw_publisher_count_matched_subscriptions
* rmw_subscription_count_matched_publishers
* rmw_get_node_names
* rmw_get_node_names_with_enclaves
* rmw_count_publishers
* rmw_count_subscribers
* rmw_count_clients
* rmw_count_services
* rmw_get_gid_for_publisher
* rmw_get_gid_for_client
* rmw_compare_gids_equal
* rmw_get_service_names_and_types
* rmw_get_publishers_info_by_topic
* rmw_get_subscriptions_info_by_topic
* rmw_get_subscriber_names_and_types_by_node
* rmw_get_publisher_names_and_types_by_node
* rmw_get_service_names_and_types_by_node
* rmw_get_client_names_and_types_by_node
* rmw_get_topic_names_and_types

### Related Zenoh-c APIs

* zc_liveliness_declare_token
* zc_liveliness_declare_subscriber
* zc_liveliness_get

## Contexts

A ROS 2 context describes a certain middleware configuration, which can contain 0 or more ROS 2 nodes.
In `rmw_zenoh_cpp`, a context maps to a Zenoh session, along with a liveliness token for the graph cache and some additional metadata.

### Related RMW APIs

* rmw_get_zero_initialized_init_options
* rmw_init_options_copy
* rmw_init_options_fini
* rmw_get_zero_initialized_context
* rmw_init
* rmw_shutdown
* rmw_context_init
* rmw_create_guard_condition
* rmw_destroy_guard_condition
* rmw_trigger_guard_condition
* rmw_create_wait_set
* rmw_destroy_wait_set
* rmw_wait

### Related Zenoh-c APIs

* zc_liveliness_declare_subscriber
* zc_liveliness_get
* z_open
* z_close
* z_undeclare_subscriber
* z_call
* z_session_check

## Nodes

A ROS 2 node can be though of as the "unit of computation" in a ROS 2 graph; usually one node performs one particular task.
Nodes may contain publishers, subscriptions, service servers, service clients, action servers, action clients, parameters, and anything else needed to do some computation.
Zenoh has no conceptual equivalent to the ROS 2 node, so `rmw_zenoh_cpp` creates no Zenoh entities when nodes are created.
When a new node is created through the RMW API, a liveliness token of type `NN` is sent out.

### Related RMW APIs

* rmw_create_node
* rmw_destroy_node
* rmw_node_get_graph_guard_condition

### Related Zenoh-c APIs

* zc_liveliness_declare_token

## Publishers

A ROS 2 publisher sends data to 0 or more connected subscriptions.
A Zenoh publisher does exactly the same thing, so ROS 2 publishers are mapped onto Zenoh publishers in `rmw_zenoh_cpp`.
When a new publisher is created, a liveliness token of type `MP` is sent out.

### Related RMW APIs

* rmw_create_publisher
* rmw_destroy_publisher
* rmw_publish
* rmw_publish_serialized_message
* rmw_borrow_loaned_message
* rmw_return_loaned_message
* rmw_publisher_wait_for_all_acked
* rmw_publisher_get_network_flow_endpoints
* rmw_publisher_event_init

### Related Zenoh-c APIs

* zc_liveliness_declare_token
* zc_publish_put_owned
* ze_declare_publication_cache
* z_declare_publisher
* z_undeclare_publisher
* z_publisher_put

## Subscriptions

A ROS 2 subscription receives data from 1 or more connected publishers.
A Zenoh subscriber does exactly the same thing, so ROS 2 subscriptions are mapped onto Zenoh subscribers in `rmw_zenoh_cpp`.
When new data arrives, a callback within `rmw_zenoh_cpp` is executed, which takes ownership of the data and signals that there is data available.
Then rmw_wait can find out that there is data available, and the data can be delivered via rmw_take.
When a new subscription is created, a liveliness token of type `MS` is sent out.

### Related RMW APIs

* rmw_create_subscription
* rmw_destroy_subscription
* rmw_take
* rmw_take_with_info
* rmw_take_sequence
* rmw_take_serialized_message
* rmw_take_serialized_message_with_info
* rmw_wait
* rmw_subscription_set_on_new_request_callback
* rmw_subscription_set_content_filter
* rmw_subscription_get_content_filter
* rmw_take_loaned_message
* rmw_take_loaned_message_with_info
* rmw_return_loaned_message_from_subscription
* rmw_subscription_get_network_flow_endpoints
* rmw_subscription_event_init

### Related Zenoh-c APIs

* zc_liveliness_declare_token
* zc_sample_payload_rcinc
* ze_declare_querying_subscriber
* z_declare_subscriber
* z_undeclare_subscriber

## Service Clients

In ROS 2, services are meant to be used for remote procedure calls that will return fairly quickly.
`rmw_zenoh_cpp` uses Zenoh queryables to implement ROS 2 services.
When a client wants to make a request, it uses the rmw API `rmw_send_request`.
`rmw_zenoh_cpp` then calls the Zenoh `z_get` function to send a query out to the network.
Assuming there is a service server listening to that queryable, it will receive the request, perform a computation, and return the result.
The result will then be made available to the client via `rmw_take_response`.
When a new service client is created, a liveliness token of type `SC` is sent out.

### Related RMW APIs

* rmw_create_client
* rmw_destroy_client
* rmw_send_request
* rwm_take_response
* rmw_take
* rmw_take_with_info
* rmw_take_sequence
* rmw_take_serialized_message
* rmw_take_serialized_message_with_info
* rmw_wait
* rmw_service_server_is_available
* rmw_client_set_on_new_response_callback

### Related Zenoh-c APIs

* zc_liveliness_declare_token
* z_get
* z_attachment_get

## Service Servers

In ROS 2, services are meant to be used for remote procedure calls that will return fairly quickly.
`rmw_zenoh_cpp` uses Zenoh queryables to implement ROS 2 services.
When a ROS 2 node wants to advertise a service to the network, it calls `rmw_create_service`.
`rmw_zenoh_cpp` uses the `z_declare_queryable` Zenoh API to create that service.
When a client request comes in, `rmw_take_request` is called to send the query to the user callback, which should perform some computation.
Once the user callback returns, `rmw_send_response` is called to send the response back to the requester.
When a new service server is created, a liveliness token of type `SS` is sent out.

### Related RMW APIs

* rmw_create_service
* rmw_destroy_service
* rmw_take_request
* rmw_send_response
* rmw_take
* rmw_take_with_info
* rmw_take_sequence
* rmw_take_serialized_message
* rmw_take_serialized_message_with_info
* rmw_wait

### Related Zenoh-c APIs

* zc_liveliness_declare_token
* z_attachment_get
* z_declare_queryable
* z_undeclare_queryable
* z_query_value
* z_query_attachment

## Quality of Service (QoS)

The ROS 2 RMW layer defines quite a few Quality of Service settings that are largely derived from DDS.
Here is an incomplete list of some of the settings and the values that they can take:

* RELIABILITY
    * RELIABLE - Data delivery is retried until it is successfully delivered.  Because Zenoh is TCP-based (by default), this is the default in `rmw_zenoh_cpp`.
    * BEST_EFFORT - Data may be dropped during delivery. Because Zenoh is TCP-based (by default), this may not work exactly the same as in DDS.
* HISTORY
    * KEEP_LAST - For subscriptions, only keep up to a maximum number of samples (defined by depth); once the maximum is reached, older samples will be lost.
    * KEEP_ALL - For subscriptions, keep all values.
* DEPTH - The maximum number of samples to keep; only comes into play when KEEP_LAST history is used.
* DURABILITY
    * VOLATILE - Samples will only be delivered to subscriptions that are active at the time of publishing.  In `rmw_zenoh_cpp`, this is implemented via `z_declare_subscriber` on the subscription side and `z_declare_publisher` on the publisher side.
    * TRANSIENT_LOCAL - "Late-joining" subscriptions will receive historical data, along with any new data.  In `rmw_zenoh_cpp`, this is implemented via `ze_declare_querying_subscriber` on the subscription side and `ze_declare_publication_cache` on the publisher side.
* LIVELINESS
    * AUTOMATIC - The "liveliness" of an entity of the system is managed by the RMW layer.  This is what `rmw_zenoh_cpp` uses.
    * MANUAL_BY_TOPIC - It is up to the application to periodically publish to a particular topic to assert liveliness.
* DEADLINE - The period at which messages are expected to be sent/received.  Currently unimplemented in `rmw_zenoh_cpp`.
* LIFESPAN - The age at which messages are expired and no longer valid.  Currently unimplemented in `rmw_zenoh_cpp`.

### Related RMW APIs

* rmw_publisher_get_actual_qos
* rmw_subscription_get_actual_qos
* rmw_client_request_publisher_get_actual_qos
* rmw_client_response_subscription_get_actual_qos
* rmw_service_request_subscription_get_actual_qos
* rmw_service_response_publisher_get_actual_qos

### Related Zenoh-c APIs

N/A

## Events

In ROS 2 terminology, "events" are out-of-band things that may happen for communication.
For instance, if a message is lost, then the RMW layer may raise an event to the upper layers to signal that fact.

Events are broken down into subscription events and publisher events:

* Subscription
    * LIVELINESS_CHANGED
    * DEADLINE_MISSED
    * QOS_INCOMPATIBLE
    * MESSAGE_LOST
    * INCOMPATIBLE_TYPE
    * MATCHED
* Publisher
    * LIVELINESS_LOST
    * DEADLINE_MISSED
    * QOS_INCOMPATIBLE
    * INCOMPATIBLE_TYPE
    * MATCHED

### Related RMW APIs

* rmw_wait
* rmw_take
* rmw_event_set_callback
* rmw_publisher_event_init
* rmw_subscription_event_init
* rmw_take_event

### Related Zenoh-c APIs

N/A

## Actions

As of 2024-02-09, there is no concept of an action at the RMW level in ROS 2.
Instead, actions are composed of several services and pub/sub.
Thus, there is no direct implementation of actions in `rmw_zenoh_cpp`.

## Security

TBD

### Notes

 - Zenoh routers (a.k.a. `zenohd`) are required mainly to allow the peers to discover each other within the subsystem.
 - Zenoh sessions are configured in `peer` mode and connect directly to the router looking for other peers. As `gossip` scouting is being used, the router is in charge to spread the discovery information across the peers of the subsystem.
 - Each Zenoh sessions discover each other and creates direct `Peer-To-Peer` connection between them and the connection remains without relying on the router.
 - By means of the `Peer-To-Peer` connection, the publisher-subscriber interaction happens: Data flows from the publishers to the subscribers.
