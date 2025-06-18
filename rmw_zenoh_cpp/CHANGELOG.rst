^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_zenoh_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.5 (2025-06-18)
------------------
* Use data() to avoid potentially dereferencing an empty vector (`#669 <https://github.com/ros2/rmw_zenoh/issues/669>`_)
* Bump Zenoh to 1.4.0 (`#658 <https://github.com/ros2/rmw_zenoh/issues/658>`_)
* fix rmw_take_serialized_message. (`#640 <https://github.com/ros2/rmw_zenoh/issues/640>`_)
* Contributors: Julien Enoch, Tomoya Fujita, Yuyuan Yuan, Øystein Sture

0.2.4 (2025-04-20)
------------------
* Change serialization format in attachment_helpers.cpp (`#605 <https://github.com/ros2/rmw_zenoh/issues/605>`_)
* Fix the comment. (`#598 <https://github.com/ros2/rmw_zenoh/issues/598>`_)
* Bump Zenoh to v1.3.2 and improve e2e reliability with HeartbeatSporadic (`#593 <https://github.com/ros2/rmw_zenoh/issues/593>`_)
* Trigger qos event callback if there are changes before registration  (`#589 <https://github.com/ros2/rmw_zenoh/issues/589>`_)
* Set wait_set->triggered flag to false (`#584 <https://github.com/ros2/rmw_zenoh/issues/584>`_)
* Add space after `id` token in `rmw_zenohd` log string (`#578 <https://github.com/ros2/rmw_zenoh/issues/578>`_)
* fix: use `std::unique_lock` to unlock correctly on Windows (`#573 <https://github.com/ros2/rmw_zenoh/issues/573>`_)
* Contributors: ChenYing Kuo (CY), Julien Enoch, Luca Cominardi, Patrick Roncagliolo, Yadunund, yellowhatter, Yuyuan Yuan

0.2.3 (2025-03-20)
------------------
* Switch to std::map for TopicTypeMap (`#565 <https://github.com/ros2/rmw_zenoh/issues/565>`_)
* Support zenoh config override (`#559 <https://github.com/ros2/rmw_zenoh/issues/559>`_)
* Align the config with the latest Zenoh. (`#557 <https://github.com/ros2/rmw_zenoh/issues/557>`_)
* Added documentation note in the code (`#541 <https://github.com/ros2/rmw_zenoh/issues/541>`_)
* fix: unlock the mutex before making get (`#539 <https://github.com/ros2/rmw_zenoh/issues/539>`_)
* Take wait_set_lock before condition_variable notification for subscriptions (`#534 <https://github.com/ros2/rmw_zenoh/issues/534>`_)
* Switch default durability to volatile (`#530 <https://github.com/ros2/rmw_zenoh/issues/530>`_)
* Added rmw_event_type_is_supported (`#525 <https://github.com/ros2/rmw_zenoh/issues/525>`_)
* Fixed windows warning (`#518 <https://github.com/ros2/rmw_zenoh/issues/518>`_)
* Config: tune some values for ROS use case, especially with large number of Nodes (>200) (`#515 <https://github.com/ros2/rmw_zenoh/issues/515>`_)
* Honor ignore_local_publications in subscription options (`#513 <https://github.com/ros2/rmw_zenoh/issues/513>`_)
* Bump zenoh-cpp to 2a127bb, zenoh-c to 3540a3c, and zenoh to f735bf5 (`#511 <https://github.com/ros2/rmw_zenoh/issues/511>`_)
* Fix calculation of current_count_change when event status is updated (`#506 <https://github.com/ros2/rmw_zenoh/issues/506>`_)
* Fix checks for invalid arguments (`#501 <https://github.com/ros2/rmw_zenoh/issues/501>`_)
* Fail creation of entities if qos contains unknown settings (`#499 <https://github.com/ros2/rmw_zenoh/issues/499>`_)
* Enable Zenoh UDP transport (`#488 <https://github.com/ros2/rmw_zenoh/issues/488>`_)
* fix: use the default destructor that automatically drops the zenoh reply/query and hence sends the final signal (`#474 <https://github.com/ros2/rmw_zenoh/issues/474>`_)
* Contributors: Alejandro Hernández Cordero, ChenYing Kuo (CY), Hugal31, Luca Cominardi, Tomoya Fujita, Yuyuan Yuan, Yadunund

0.2.2 (2025-02-19)
------------------
* Introduce the advanced publisher and subscriber (`#469 <https://github.com/ros2/rmw_zenoh/issues/469>`_)
* Add tracing instrumentation using tracetools  (`#471 <https://github.com/ros2/rmw_zenoh/issues/471>`_)
* Switch to debug log if topic_name not in topic_map (`#464 <https://github.com/ros2/rmw_zenoh/issues/464>`_)
* Bump Zenoh to commit id 3bbf6af (1.2.1 + few commits) (`#461 <https://github.com/ros2/rmw_zenoh/issues/461>`_)
* Contributors: Alejandro Hernandez Cordero, Christophe Bedard, Julien Enoch, Yadunund, Yuyuan Yuan

0.2.1 (2025-02-04)
------------------
* Bump Zenoh to commit id e4ea6f0 (1.2.0 + few commits) (`#448 <https://github.com/ros2/rmw_zenoh/issues/448>`_)
* Inform users that peers will not discover and communicate with one another until the router is started (`#444 <https://github.com/ros2/rmw_zenoh/issues/444>`_)
* Clear the error after rmw_serialized_message_resize() (`#436 <https://github.com/ros2/rmw_zenoh/issues/436>`_)
* Fix `ZENOH_ROUTER_CHECK_ATTEMPTS` which was not respected (`#428 <https://github.com/ros2/rmw_zenoh/issues/428>`_)
* fix: use the default destructor to drop the member `Payload` (`#420 <https://github.com/ros2/rmw_zenoh/issues/420>`_)
* Remove `gid_hash\_` from `AttachmentData` (`#417 <https://github.com/ros2/rmw_zenoh/issues/417>`_)
* Sync the config with the default config in Zenoh. (`#413 <https://github.com/ros2/rmw_zenoh/issues/413>`_)
* fix: check the context validity before accessing the session (`#406 <https://github.com/ros2/rmw_zenoh/issues/406>`_)
* Fix wan't typo (`#401 <https://github.com/ros2/rmw_zenoh/issues/401>`_)
* Contributors: ChenYing Kuo (CY), Chris Lalancette, Julien Enoch, Mahmoud Mazouz, Tim Clephas, Yadunund, Yuyuan Yuan

0.2.0 (2025-01-02)
------------------
* An alternative middleware for ROS 2 based on Zenoh.
* Contributors: Alejandro Hernández Cordero, Alex Day, Bernd Pfrommer, ChenYing Kuo (CY), Chris Lalancette, Christophe Bedard, CihatAltiparmak, Esteve Fernandez, Franco Cipollone, Geoffrey Biggs, Hans-Martin, James Mount, Julien Enoch, Morgan Quigley, Nate Koenig, Shivang Vijay, Yadunund, Yuyuan Yuan, methylDragon
