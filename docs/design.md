# Design

## Network Topology

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

### Notes

 - Zenoh routers (a.k.a. `zenohd`) are required mainly to allow the peers to discover each other within the subsystem.
 - Zenoh sessions are configured in `peer` mode and connect directly to the router looking for other peers. As `gossip` scouting is being used, the router is in charge to spread the discovery information across the peers of the subsystem.
 - Each Zenoh sessions discover each other and creates direct `Peer-To-Peer` connection between them and the connection remains without relying on the router.
 - By means of the `Peer-To-Peer` connection, the publisher-subscriber interaction happens: Data flows from the publishers to the subscribers.
