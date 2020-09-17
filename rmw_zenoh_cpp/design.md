# ROS concepts required to be supported by an RMW implementation

Broadly speaking, any RMW implementation must provide the following functionality.

- Create a participant in the network to which the RMW implementation connects
- One-way communication of messages across a topic
  - Publication of messages to a specified topic
  - Subscription to a specified topic to receive messages
- Remote Procedure Call (RPC) two-way communication across a service
  - Publication of messages to a specified topic, with a sequence number attached
  - Subscription to a specified topic to receive messages, extracting a sequence number
- Wait for events to occur in the middleware
- Provide QoS for topics and services
- Get information about the participants in the ROS graph
  - Number of publishers and subscribers on a topic
  - Number of subscribers listening to a publisher, and number of publishers publishing to a subscriber
  - All known namespaces and nodes
  - Liveliness of nodes and publishers
  - Availability of a service provider
  - Actual QoS used on a publisher or subscriber
- Serialise and deserialise data using the RMW implementation's serialisation method
  - Publish and receive serialised data as-is
- [Optional] Ability to borrow message memory from the RMW implementation, so as to avoid memory copies
- [Optional?] Security enclaves


# Relationship between Zenoh concepts and ROS concepts

The Zenoh-based RMW library implements the required RMW concepts as described in the following sections.

## Topics

## Services

## Wait sets

## QoS

## Graph information

## Data serialisation and deserialisation

## Message loaning

## Security

What security?


# Possible future extensions

There are some additional ROS concepts that the RMW layer is not required to provide, but which can potentially be implemented using Zenoh concepts in a more natural way than what the RMW API currently provides.

## Actions

_Note: this section is speculative as parameters are currently implemented in the rcl layer using topics and services._

## Parameters

_Note: this section is speculative as parameters are currently implemented in the rcl layer using topics and services._

