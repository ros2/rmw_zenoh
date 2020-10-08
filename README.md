# rmw_zenoh

![](https://github.com/osrf/rmw_zenoh/workflows/CI/badge.svg)

`rmw_zenoh` provides an experimental implementation of the ROS middleware interface using Eclipse Zenoh as the middleware.
The RMW implementation is contained in the `rmw_zenoh_cpp` package.

*Note that this implementation is currently very young, under heavy development, and generally rapidly changing.*
*It may work smoothly, or it may blow up and squash your cat.*
*Please do not expect to use it except in an experimental capacity.*

Zenoh does not include its own serialisation scheme.
Currently a hacked-up copy of [Fast CDR]() is used.
In the future we hope to support different serialisation schemes, possibly in a configurable way.


## Installation

Set up a new workspace using the [provided `rmw_zenoh.repos` file](https://raw.githubusercontent.com/osrf/rmw_zenoh/main/rmw_zenoh.repos).

```shell
mkdir -p ~/rmw_zenoh_ws/src
cd ~/rmw_zenoh_ws
wget https://raw.githubusercontent.com/osrf/rmw_zenoh/main/rmw_zenoh.repos
vcs import src < rmw_zenoh.repos
```

Next, prepare your environment for compiling Zenoh.
The `rmw_zenoh_cpp` package depends on the `zenoh_vendor` package to pull in Zenoh.
`zenoh_vendor` provides the Zenoh foreign function interface (for C) library from the [Rust implementation of Zenoh](https://github.com/eclipse-zenoh/zenoh/tree/rust-master).
The library is obtained by cloning and compiling the Zenoh source code from GitHub.
Because it will compile Zenoh automatically, you must meet [the preconditions for compiling Zenoh](https://github.com/eclipse-zenoh/zenoh/tree/rust-master#how-to-build-it).
Most importantly, you must use the nightly version of the Rust toolchain.
[Install Rust using `rustup`](https://rustup.rs/), then run the following command to enable it.

```shell
rustup default nightly
```

Next, ensure all the dependencies of the packages in the `rmw_zenoh_ws` are available.

```shell
cd ~/rmw_zenoh_ws
rosdep install --ignore-src --from-paths src --rosdistro=foxy -y
```

You are now ready to compile the workspace.
Because of an as-yet unresolved quirk in the typesupport compilation, this needs to be done twice.

```shell
cd ~/rmw_zenoh_ws
colcon build
source install/setup.bash
colcon build --cmake-force-configure
```

## Testing

You can test `rmw_zenoh_cpp` using the existing ROS 2 sample nodes.

Open two terminals and source the `rmw_zenoh_ws` workspace in each one.
Then, in each terminal, change the active RMW implementation to `rmw_zenoh_cpp` using the `RMW_IMPLEMENTATION` environment variable.
(See [this tutorial](https://index.ros.org/doc/ros2/Tutorials/Working-with-multiple-RMW-implementations/#specifying-rmw-implementations) for more information on changing your active RMW implementation.)
Then, launch the demonstration publisher and subscriber nodes, one in each terminal.

Terminal 1:

```shell
cd ~/rmw_zenoh_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run demo_nodes_cpp talker
```

Terminal 2:

```shell
cd ~/rmw_zenoh_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run demo_nodes_cpp listener
```

You should see data being transmitted from the `talker` program to the `listener` program.
You may also see various informational messages from the `rmw_zenoh_cpp` implementation.
Most of these are temporary aides to development, but they do indicate that the correct RMW implementation is being used.
