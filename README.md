# rmw_zenoh

[![build](https://github.com/ros2/rmw_zenoh/actions/workflows/build.yaml/badge.svg)](https://github.com/ros2/rmw_zenoh/actions/workflows/build.yaml)
[![style](https://github.com/ros2/rmw_zenoh/actions/workflows/style.yaml/badge.svg)](https://github.com/ros2/rmw_zenoh/actions/workflows/style.yaml)

A ROS 2 RMW implementation based on Zenoh that is written using the zenoh-c bindings.

## Design

For information about the Design please visit [design](docs/design.md) page.

## Requirements
- [ROS 2](https://docs.ros.org): Rolling/Iron


## Setup

Install latest rustc.
> Note: The version of rustc that can be installed via apt is outdated.
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Build `rmw_zenoh_cpp`

```bash
mkdir ~/ws_rmw_zenoh/src -p && cd ~/ws_rmw_zenoh/src
git clone https://github.com/ros2/rmw_zenoh.git
cd ~/ws_rmw_zenoh
source /opt/ros/<DISTRO>/setup.bash # replace <DISTRO> with ROS 2 distro of choice
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

```

## Test

Make sure to source the built workspace using the commands below prior to running any other commands.
```bash
cd ~/ws_rmw_zenoh
source install/setup.bash
```

### Start the zenoh router
> Note: Manually launching zenoh router won't be necessary in the future.
```bash
# terminal 1
ros2 run rmw_zenoh_cpp init_rmw_zenoh_router
```

> Note: Without the zenoh router, nodes will not be able to discover each other since multicast discovery is disabled by default in the node's session config. Instead, nodes will receive discovery information about other peers via the zenoh router's gossip functionality. See more information on the session configs [below](#config).

### Run the `talker`
```bash
# terminal 2
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run demo_nodes_cpp talker
```
> Note: Ignore all the warning printouts.

### Run the `listener`
```bash
# terminal 2
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run demo_nodes_cpp listener
```

The listener node should start receiving messages over the `/chatter` topic.
> Note: Ignore all the warning printouts.

### Graph introspection
Presently we only support limited `ros2cli` commands to introspect the ROS graph such as `ros2 node list` and `ros2 topic list`.

## Config
The [default configuration](rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5) sets up the zenoh sessions with the following main characteristics:

Table:
| Zenoh Config | Default |
| :---:   | :---: |
| udp_multicast | disabled |
| gossip scouting | enabled |
| connect | tcp/localhost:7447 |

This assumes that there is a `zenohd` running in the system at port 7447.
A custom configuration may be provided by setting the `RMW_ZENOH_CONFIG_FILE` environment variable to point to a custom zenoh configuration file.


## TODO Features
- [x] Publisher
- [x] Subscription
- [ ] Client
- [ ] Service
- [ ] Graph introspection
