# rmw_alternative

![](https://github.com/yadunund/rmw_alternative/workflows/build/badge.svg)
![](https://github.com/yadunund/rmw_alternative/workflows/style/badge.svg)

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
mkdir ~/ws_rmw_alternative/src -p && cd ~/ws_rmw_alternative/src
git clone git@github.com:yadunund/rmw_alternative
cd ~/ws_rmw_alternative
source /opt/ros/<DISTRO>/setup.bash # replace <DISTRO> with ROS 2 distro of choice
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

```

## Test

Source workspace
```bash
cd ~/ws_rmw_alternative
source install/setup.bash
```

In a terminal launch Zenoh router:
```bash
ros2 run rmw_zenoh_cpp init_rmw_zenoh_router
```
> Note: Manually launching zenoh router won't be necessary in the future.

In a different terminal source install folder and execute:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic pub "/chatter" std_msgs/msg/String '{data: hello}'
```

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
- [ ] Subscription
- [ ] Client
- [ ] Service
- [ ] Graph introspection
