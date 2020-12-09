^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_ecal_dynamic_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2020-10-19)
------------------
* Added support for Foxy Fitzroy

0.4.1 (2020-10-05)
------------------
* Performance improvements for messages containing primitive type arrays
* Bug fixes

0.4.0 (2020-08-13)
------------------
* Subscribers will now receive messages from publishers that are running in same process
* Fixed bug where deserializing empty arrays/strings would crash the node
* Updated typenames naming convention, type names will be named like this "namespace/namespace2/type" for custom serialization and proto:typename for protobuf serialization
* Rosbag should be able to record/play topics now if they are using custom serialization
Known issues:
* Due to bug in eCAL5 where service calls aren't asynchronous, services in RMW aren't 
  async either, also this means that actions won't behave as expected where all of their streamed messages
  would get propagated to client when action request finishes.

0.3.0 (2020-06-24)
------------------
* Implemented client/service support
* Implemented actions support
* Implemented quality of life functions support (eg. graph querying functions)
* Implemented QOS support (eg. graph querying functions)
* Added support for Dashing Diadema
* Updated topic/service naming prefixes to follow ros2 rmw conventions
Known issues:
* Due to bug in eCAL5 where service calls aren't asynchronous, services in RMW aren't 
  async either, also this means that actions won't behave as expected where all of their streamed messages
  would get propagated to client when action request finishes.
* Due to internal typename naming, rosbag recording/playing tools won't work, eCAL equivalents should work fine.

0.2.0 (2020-03-24)
------------------
* Renamed package to rmw_ecal_dynamic_cpp
* Implemented protobuf serialization
* Bug fixes
* Performance improvements

0.1.0 (2020-03-03)
------------------
* Implemented pub/sub support
* Implemented custom type support
