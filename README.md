# rmw_zenoh

`rmw_zenoh` provides an experimental implementation of the ROS middleware interface using Eclipse Zenoh as the middleware.
The RMW implementation is contained in the `rmw_zenoh` package.

Zenoh does not include its own marshalling.
The library focuses solely on getting binary blobs from one place to another.


## Testing

To test this RMW implementation, create a new workspace and add this repository to it, then build the workspace.

```shell
mkdir -p zenoh/src
cd zenoh/src
git clone https://github.com/osrf/rmw_zenoh.git
cd ../..
colcon build
```

Then, after sourcing the workspace, open two terminals and launch the demonstration publisher and subscriber nodes, prefixing with the `RMW_IMPLEMENTATION` environment variable set to use the Zenoh RMW library.

```shell
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run demo_nodes_cpp talker
```

```shell
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run demo_nodes_cpp listener
```

At this stage, you should see the logged output of the RMW functions being called, followed rapidly by a failure due to almost all the functions being stubbed out and returning error values.
