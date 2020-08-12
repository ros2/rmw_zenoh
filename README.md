# rmw_zenoh

`rmw_zenoh` provides an experimental implementation of the ROS middleware interface using Eclipse Zenoh as the middleware.
The RMW implementation is contained in the `rmw_zenoh` package.

Zenoh does not include its own marshalling.
The library focuses solely on getting binary blobs from one place to another.


## Testing

First, you'll need a build of `libzenoh_ffi.so` for your machine.
That is currently outside the scope of this document because it involves Rust and stuff.

To test this RMW implementation, first we will create a workspace with "stock" ROS 2 Foxy in it.
Then we'll overlay a much smaller workspace with the Zenoh-specific things, and other packages we need to rebuild.
This will make iteration much more pleasant.

```shell
mkdir -p ~/ros2_foxy/src
cd ~/ros2_foxy
wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
vcs import src < ros2.repos
colcon build
# during this process, it is recommended to walk away and refill beverage, etc.
```

```shell
mkdir -p ~/zenoh_ws/src
cd ~/zenoh_ws/src
git clone https://github.com/ros2/common_interfaces
git clone https://github.com/ros2/rcl_interfaces
git clone https://github.com/methylDragon/rosidl_typesupport_zenoh
git clone https://github.com/methylDragon/zenoh_ros_examples
git clone ssh://git@github.com/methylDragon/rmw_zenoh.git -b develop
mkdir rmw_zenoh/zenoh_ament/lib
ln -s LOCATION_OF_ZENOH_LIBRARY.so ~/zenoh_ws/src/rmw_zenoh/zenoh_ament/lib/libzenoh_ffi.so
cd ~/zenoh_ws
source ~/ros2_foxy/install/setup.bash
colcon build
colcon build --cmake-force-configure
```

Then, after sourcing the workspace, open two terminals and launch the demonstration publisher and subscriber nodes, prefixing with the `RMW_IMPLEMENTATION` environment variable set to use the Zenoh RMW library.

```shell
cd ~/zenoh_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run demo_nodes_cpp talker
```

```shell
cd ~/zenoh_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run demo_nodes_cpp listener
```

At this stage, you should see the logged output of the RMW functions being called, followed rapidly by a failure due to almost all the functions being stubbed out and returning error values.
