^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_zenoh_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.4 (2026-04-30)
-------------------

0.10.3 (2026-04-14)
-------------------

* Bump Zenoh to 1.8.0, fix Windows shutdown hang, and resolve synchronization with ``undeclare`` (`#964 <https://github.com/ros2/rmw_zenoh/issues/964>`_)
* Revert changes to build against rust >= 1.75 and bump zenoh to 1.8.0 (`#960 <https://github.com/ros2/rmw_zenoh/issues/960>`_)
* Prevent deadlock by not holding both locks when processing event data (`#937 <https://github.com/ros2/rmw_zenoh/issues/937>`_)
* Bump zenoh to 1.8.0 (`#935 <https://github.com/ros2/rmw_zenoh/issues/935>`_)
* Explicitly set `false` for the content filtering feature (`#938 <https://github.com/ros2/rmw_zenoh/issues/938>`_)
* Add deadline/liveliness QoS events to ``rmw_zenoh_cpp`` (`#934 <https://github.com/ros2/rmw_zenoh/issues/934>`_)
* Catch ``PackageNotFoundError`` during default config URI loading to prevent crash (`#915 <https://github.com/ros2/rmw_zenoh/issues/915>`_)
* Populate ``reception_sequence_number`` and ``advertise_sequence_number`` features (`#920 <https://github.com/ros2/rmw_zenoh/issues/920>`_)
* Use ``get_package_share_path`` (`#913 <https://github.com/ros2/rmw_zenoh/issues/913>`_)
* Address outstanding TODO items (`#896 <https://github.com/ros2/rmw_zenoh/issues/896>`_)
* Expose zenoh session (`#865 <https://github.com/ros2/rmw_zenoh/issues/865>`_)
* Fix config loading with incorrect path variable (`#898 <https://github.com/ros2/rmw_zenoh/issues/898>`_)
* Fix build binary workflow (`#895 <https://github.com/ros2/rmw_zenoh/issues/895>`_)
* Fix line ending in session open error message (`#888 <https://github.com/ros2/rmw_zenoh/issues/888>`_)
* Update deprecated ``ament_index_cpp`` API (`#879 <https://github.com/ros2/rmw_zenoh/issues/879>`_)
* Remove ``default`` from switch with enum to enable compiler warnings (`#871 <https://github.com/ros2/rmw_zenoh/issues/871>`_)
* Use shared SHM transport provider instead of creating a new instance (`#857 <https://github.com/ros2/rmw_zenoh/issues/857>`_)
* Bump ``zenoh`` to 1.7.1 (`#870 <https://github.com/ros2/rmw_zenoh/issues/870>`_)

* Contributors: Alejandro Hernández Cordero, Hervé Audren, Julien Enoch, Nikola Banović, Shane Loretz, Skyler Medeiros, Tomoya Fujita, Yuyuan Yuan, jordanburklund, yellowhatter

0.10.2 (2025-11-18)
-------------------
* Add rmw_get_clients_info_by_service and rmw_get_servers_info_by_service (`#679 <https://github.com/ros2/rmw_zenoh/issues/679>`_)
* Fix REP url locations (`#858 <https://github.com/ros2/rmw_zenoh/issues/858>`_)
* Contributors: Minju, Lee, Tim Clephas

0.10.1 (2025-11-12)
-------------------
* Restore ZENOH_CONFIG_OVERRIDE after isolation is finished (`#855 <https://github.com/ros2/rmw_zenoh/issues/855>`_)
* Fix typo in 'triggered' (`#844 <https://github.com/ros2/rmw_zenoh/issues/844>`_)
* Log details at SHM creation (alloc and threashold sizes) (`#835 <https://github.com/ros2/rmw_zenoh/issues/835>`_)
* Contributors: Alejandro Hernandez Cordero, Christophe Bedard, Julien Enoch, Scott K Logan

0.10.0 (2025-10-04)
-------------------
* Change default value of ZENOH_SHM_ALLOC_SIZE to 48 MiB (`#830 <https://github.com/ros2/rmw_zenoh/issues/830>`_)
* config: increase queries_default_timeout to 10min (`#820 <https://github.com/ros2/rmw_zenoh/issues/820>`_)
* Fix compile with clang (`#819 <https://github.com/ros2/rmw_zenoh/issues/819>`_)
* feat(logging): add contextual information to log messages (`#809 <https://github.com/ros2/rmw_zenoh/issues/809>`_)
* Align the config with upstream Zenoh. (`#785 <https://github.com/ros2/rmw_zenoh/issues/785>`_)
* fix: resolve memory leak when publishing with the default allocator (`#797 <https://github.com/ros2/rmw_zenoh/issues/797>`_)
* Contributors: ChenYing Kuo (CY), Julien Enoch, Yuyuan Yuan, Yadunund

0.9.1 (2025-09-10)
------------------
* Recycle serialization buffers on transmission (`#342 <https://github.com/ros2/rmw_zenoh/issues/342>`_)
* refactor: avoid redundant key expression creation when replying (`#732 <https://github.com/ros2/rmw_zenoh/issues/732>`_)
* Contributors: Chris Lalancette, Yadunund, Mahmoud Mazouz, Yuyuan Yuan, Julien Enoch

0.9.0 (2025-08-21)
------------------
* Do not include rosidl_typesupport\_{c,cpp} in rmw impl typesupport list (`#748 <https://github.com/ros2/rmw_zenoh/issues/748>`_)
* fixing typo flow to flows in config files (`#740 <https://github.com/ros2/rmw_zenoh/issues/740>`_)
* Shared Memory on C++ API (`#363 <https://github.com/ros2/rmw_zenoh/issues/363>`_)
* Bump Zenoh to v1.5.0 (`#728 <https://github.com/ros2/rmw_zenoh/issues/728>`_)
* Contributors: ChenYing Kuo (CY), Christophe Bedard, Faseel Chemmadan, Julien Enoch, milidam, Steven Palma, Yadunund, yellowhatter, Yuyuan Yuan

0.8.2 (2025-07-29)
------------------
* rmw_zenoh_cpp: Include algorithm for std::find_if (`#723 <https://github.com/ros2/rmw_zenoh/issues/723>`_)
* Use rfind to avoid issues with service types ending in Request or Response (`#719 <https://github.com/ros2/rmw_zenoh/issues/719>`_)
* Remove the extra copy on the publisher side (`#711 <https://github.com/ros2/rmw_zenoh/issues/711>`_)
* Avoid ambiguity with variable shadowing (`#706 <https://github.com/ros2/rmw_zenoh/issues/706>`_)
* Only configure the timeout of the action-related service `get_result` to maximum value. (`#685 <https://github.com/ros2/rmw_zenoh/issues/685>`_)
* Use Zenoh Querier to replace Session.get (`#694 <https://github.com/ros2/rmw_zenoh/issues/694>`_)
* Contributors: ChenYing Kuo (CY), Filip, Jan Vermaete, yadunund

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
