# rmw_zenoh

`rmw_zenoh` provides an experimental implementation of the ROS middleware interface using Eclipse Zenoh as the middleware.
The RMW implementation is contained in the `rmw_zenoh` package.

Zenoh does not include its own marshalling.
The library focuses solely on getting binary blobs from one place to another.
