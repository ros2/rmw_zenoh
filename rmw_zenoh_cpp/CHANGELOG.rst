^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_zenoh_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.1 (2025-07-01)
------------------

0.8.0 (2025-06-18)
------------------
* Use data() to avoid potentially dereferencing an empty vector (`#667 <https://github.com/ros2/rmw_zenoh/issues/667>`_)
* Bump Zenoh to 1.4.0 (`#652 <https://github.com/ros2/rmw_zenoh/issues/652>`_)
* Contributors: Julien Enoch, Øystein Sture

0.7.1 (2025-05-19)
------------------
* fix(comment): correct the QoS incompatibilities (`#644 <https://github.com/ros2/rmw_zenoh/issues/644>`_)
* fix rmw_take_serialized_message. (`#638 <https://github.com/ros2/rmw_zenoh/issues/638>`_)
* Update CMakeLists.txt (`#617 <https://github.com/ros2/rmw_zenoh/issues/617>`_)
* Contributors: Alejandro Hernández Cordero, Tomoya Fujita, Yuyuan Yuan, mosfet80

0.7.0 (2025-04-24)
------------------

0.6.0 (2025-04-18)
------------------
* Change serialization format in attachment_helpers.cpp (`#601 <https://github.com/ros2/rmw_zenoh/issues/601>`_)
* Bump Zenoh to v1.3.2 and improve e2e reliability with HeartbeatSporadic (`#591 <https://github.com/ros2/rmw_zenoh/issues/591>`_)
* Implement rmw_test_fixture to start the Zenoh router (`#583 <https://github.com/ros2/rmw_zenoh/issues/583>`_)
* Add quality declaration (`#483 <https://github.com/ros2/rmw_zenoh/issues/483>`_)
* Trigger qos event callback if there are changes before registration  (`#587 <https://github.com/ros2/rmw_zenoh/issues/587>`_)
* Set wait_set->triggered flag to false (`#575 <https://github.com/ros2/rmw_zenoh/issues/575>`_)
* Add space after `id` token in `rmw_zenohd` log string (`#576 <https://github.com/ros2/rmw_zenoh/issues/576>`_)
* Use `std::unique_lock` to unlock correctly on Windows (`#570 <https://github.com/ros2/rmw_zenoh/issues/570>`_)
* Contributors: Alejandro Hernández Cordero, ChenYing Kuo (CY), Julien Enoch, Luca Cominardi, Patrick Roncagliolo, Scott K Logan, Yuyuan Yuan, yadunund, yellowhatter

0.5.0 (2025-03-20)
------------------
* Switch to std::map for TopicTypeMap (`#546 <https://github.com/ros2/rmw_zenoh/issues/546>`_)
* Support zenoh config override (`#551 <https://github.com/ros2/rmw_zenoh/issues/551>`_)
* Align the config with the latest Zenoh. (`#556 <https://github.com/ros2/rmw_zenoh/issues/556>`_)
* Added documentation note in the code (`#540 <https://github.com/ros2/rmw_zenoh/issues/540>`_)
* fix: unlock the mutex before making get (`#537 <https://github.com/ros2/rmw_zenoh/issues/537>`_)
* Take wait_set_lock before condition_variable notification for subscriptions (`#528 <https://github.com/ros2/rmw_zenoh/issues/528>`_)
* Switch default durability to volatile (`#521 <https://github.com/ros2/rmw_zenoh/issues/521>`_)
* Added rmw_event_type_is_supported (`#502 <https://github.com/ros2/rmw_zenoh/issues/502>`_)
* Fixed windows warning (`#500 <https://github.com/ros2/rmw_zenoh/issues/500>`_)
* Config: tune some values for ROS use case, especially with large number of Nodes (>200) (`#509 <https://github.com/ros2/rmw_zenoh/issues/509>`_)
* Honor ignore_local_publications in subscription options (`#508 <https://github.com/ros2/rmw_zenoh/issues/508>`_)
* Bump zenoh-cpp to 2a127bb, zenoh-c to 3540a3c, and zenoh to f735bf5 (`#503 <https://github.com/ros2/rmw_zenoh/issues/503>`_)
* Fix calculation of current_count_change when event status is updated (`#504 <https://github.com/ros2/rmw_zenoh/issues/504>`_)
* Fix checks for invalid arguments (`#497 <https://github.com/ros2/rmw_zenoh/issues/497>`_)
* Fail creation of entities if qos contains unknown settings (`#494 <https://github.com/ros2/rmw_zenoh/issues/494>`_)
* use rmw_enclave_options_xxx APIs instead. (`#491 <https://github.com/ros2/rmw_zenoh/issues/491>`_)
* Enable Zenoh UDP transport (`#486 <https://github.com/ros2/rmw_zenoh/issues/486>`_)
* fix: use the default destructor that automatically drops the zenoh reply/query and hence sends the final signal (`#473 <https://github.com/ros2/rmw_zenoh/issues/473>`_)
* Contributors: Alejandro Hernández Cordero, ChenYing Kuo (CY), Hugal31, Luca Cominardi, Tomoya Fujita, Yuyuan Yuan, Yadunund

0.4.0 (2025-02-19)
------------------
* Introduce the advanced publisher and subscriber (`#368 <https://github.com/ros2/rmw_zenoh/issues/368>`_)
* Switch to debug log if topic_name not in topic_map (`#454 <https://github.com/ros2/rmw_zenoh/issues/454>`_)
* Bump Zenoh to commit id 3bbf6af (1.2.1 + few commits) (`#456 <https://github.com/ros2/rmw_zenoh/issues/456>`_)
* Contributors: Julien Enoch, Yuyuan Yuan, Yadunund

0.3.1 (2025-02-04)
------------------
* Bump Zenoh to commit id e4ea6f0 (1.2.0 + few commits) (`#446 <https://github.com/ros2/rmw_zenoh/issues/446>`_)
* Inform users that peers will not discover and communicate with one another until the router is started (`#440 <https://github.com/ros2/rmw_zenoh/issues/440>`_)
* Clear the error after rmw_serialized_message_resize() (`#435 <https://github.com/ros2/rmw_zenoh/issues/435>`_)
* Fix `ZENOH_ROUTER_CHECK_ATTEMPTS` which was not respected (`#427 <https://github.com/ros2/rmw_zenoh/issues/427>`_)
* fix: use the default destructor to drop the member `Payload` (`#419 <https://github.com/ros2/rmw_zenoh/issues/419>`_)
* Remove `gid_hash\_` from `AttachmentData` (`#416 <https://github.com/ros2/rmw_zenoh/issues/416>`_)
* Sync the config with the default config in Zenoh. (`#396 <https://github.com/ros2/rmw_zenoh/issues/396>`_)
* fix: check the context validity before accessing the session (`#403 <https://github.com/ros2/rmw_zenoh/issues/403>`_)
* Fix wan't typo (`#400 <https://github.com/ros2/rmw_zenoh/issues/400>`_)
* Contributors: ChenYing Kuo (CY), Chris Lalancette, Julien Enoch, Mahmoud Mazouz, Tim Clephas, Yuyuan Yuan, Yadunund

0.3.0 (2025-01-02)
------------------
* An alternative middleware for ROS 2 based on Zenoh.
* Contributors: Alejandro Hernández Cordero, Alex Day, Bernd Pfrommer, ChenYing Kuo (CY), Chris Lalancette, Christophe Bedard, CihatAltiparmak, Esteve Fernandez, Franco Cipollone, Geoffrey Biggs, Hans-Martin, James Mount, Julien Enoch, Morgan Quigley, Nate Koenig, Shivang Vijay, Yadunund, Yuyuan Yuan, methylDragon
