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
rosdep install --from-paths src --ignore-src --rosdistro <DISTRO> -y # replace <DISTRO> with ROS 2 distro of choice
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
ros2 run rmw_zenoh_cpp rmw_zenohd
```

> Note: Without the zenoh router, nodes will not be able to discover each other since multicast discovery is disabled by default in the node's session config. Instead, nodes will receive discovery information about other peers via the zenoh router's gossip functionality. See more information on the session configs [below](#config).

### Run the `talker`
```bash
# terminal 2
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run demo_nodes_cpp talker
```

### Run the `listener`
```bash
# terminal 2
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run demo_nodes_cpp listener
```

The listener node should start receiving messages over the `/chatter` topic.

## Configuration
`rmw_zenoh` relies on separate configurations files to configure the Zenoh `router` and `session` respectively.
To understand more about `routers` and `sessions`, see [Zenoh documentation](https://zenoh.io/docs/getting-started/deployment/).
For more information on the topology of Zenoh adopted in `rmw_zenoh`, please see [Design](#design).
Default configuration files are used by `rmw_zenoh` however certain environment variables may be set to provide absolute paths to custom configuration files.
The table below summarizes the default files and the environment variables for the `router` and `session`.
For a complete list of configurable parameters, see [zenoh/DEFAULT_CONFIG.json5](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5).

|         |                                            Default config                                            |   Envar for custom config  |
|---------|:----------------------------------------------------------------------------------------------------:|:--------------------------:|
| Router  |  [DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5](rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5)  |  `ZENOH_ROUTER_CONFIG_URI` |
| Session | [DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5](rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5) | `ZENOH_SESSION_CONFIG_URI` |

For example, to set the path to a custom `router` configuration file,
```bash
export ZENOH_ROUTER_CONFIG_URI=$HOME/MY_ZENOH_ROUTER_CONFIG.json5
```

### Connecting multiple hosts
By default, all discovery traffic is local per host, where the host is the PC running a Zenoh `router`.
To bridge communications across two hosts, the `router` configuration for one the hosts must be updated to connect to the other `router` at startup.
This is done by specifying an endpoint in host's `router` configuration file to as seen below.
In this example, the `router` will connect to the `router` running on a second host with IP address `192.168.1.1` and port `7447`.

```json
{
  connect: {
    endpoints: ["tcp/192.168.1.1:7447"],
  },
}
```

> Note: To connect multiple hosts, include the endpoints of all routers in the network.
