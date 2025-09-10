^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_zenoh_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.6 (2025-09-10)
------------------
* Recycle serialization buffers on transmission (`#768 <https://github.com/ros2/rmw_zenoh/issues/768>`_)
* refactor: avoid redundant key expression creation when replying (`#756 <https://github.com/ros2/rmw_zenoh/issues/756>`_)
* Contributors: Chris Lalancette, Yadunund, Mahmoud Mazouz, Yuyuan Yuan, Julien Enoch

0.1.5 (2025-08-21)
------------------
* fixing typo flow to flows in config files (`#747 <https://github.com/ros2/rmw_zenoh/issues/747>`_)
* Shared Memory on C++ API (`#743 <https://github.com/ros2/rmw_zenoh/issues/743>`_)
* Bump Zenoh to v1.5.0 (`#737 <https://github.com/ros2/rmw_zenoh/issues/737>`_)
* rmw_zenoh_cpp: Include algorithm for std::find_if (`#727 <https://github.com/ros2/rmw_zenoh/issues/727>`_)
* Use rfind to avoid issues with service types ending in Request or Response (`#722 <https://github.com/ros2/rmw_zenoh/issues/722>`_)
* Remove the extra copy on the publisher side (`#714 <https://github.com/ros2/rmw_zenoh/issues/714>`_)
* Contributors: ChenYing Kuo (CY), Christophe Bedard, Faseel Chemmadan, Filip, Jan Vermaete, Julien Enoch, milidam, Steven Palma, Yadunund, yellowhatter, Yuyuan Yuan

0.1.4 (2025-07-11)
------------------
* Avoid ambiguity with variable shadowing (`#706 <https://github.com/ros2/rmw_zenoh/issues/706>`_) (`#709 <https://github.com/ros2/rmw_zenoh/issues/709>`_)
* Only configure the timeout of the action-related service `get_result` to maximum value. (`#685 <https://github.com/ros2/rmw_zenoh/issues/685>`_) (`#703 <https://github.com/ros2/rmw_zenoh/issues/703>`_)
* Use Zenoh Querier to replace Session.get (`#694 <https://github.com/ros2/rmw_zenoh/issues/694>`_) (`#699 <https://github.com/ros2/rmw_zenoh/issues/699>`_)
* Contributors: ChenYing Kuo (CY), Yadunund

0.1.3 (2025-06-26)
------------------

0.1.2 (2025-06-19)
------------------
* Use data() to avoid potentially dereferencing an empty vector (`#667 <https://github.com/ros2/rmw_zenoh/issues/667>`_) (`#670 <https://github.com/ros2/rmw_zenoh/issues/670>`_)
* Bump Zenoh to 1.4.0 (`#652 <https://github.com/ros2/rmw_zenoh/issues/652>`_) (`#659 <https://github.com/ros2/rmw_zenoh/issues/659>`_)
* fix(comment): correct the QoS incompatibilities (`#644 <https://github.com/ros2/rmw_zenoh/issues/644>`_) (`#647 <https://github.com/ros2/rmw_zenoh/issues/647>`_)
* Change serialization format in attachment_helpers.cpp (backport `#601 <https://github.com/ros2/rmw_zenoh/issues/601>`_) (`#606 <https://github.com/ros2/rmw_zenoh/issues/606>`_)
* Fix the comment. (`#597 <https://github.com/ros2/rmw_zenoh/issues/597>`_) (`#599 <https://github.com/ros2/rmw_zenoh/issues/599>`_)
* Bump Zenoh to v1.3.2 and improve e2e reliability with HeartbeatSporadic (`#591 <https://github.com/ros2/rmw_zenoh/issues/591>`_) (`#594 <https://github.com/ros2/rmw_zenoh/issues/594>`_)
* Trigger qos event callback if there are changes before registration  (backport `#587 <https://github.com/ros2/rmw_zenoh/issues/587>`_) (`#590 <https://github.com/ros2/rmw_zenoh/issues/590>`_)
* Set wait_set->triggered flag to false (`#575 <https://github.com/ros2/rmw_zenoh/issues/575>`_) (`#585 <https://github.com/ros2/rmw_zenoh/issues/585>`_)
* Add space after `id` token in `rmw_zenohd` log string (`#576 <https://github.com/ros2/rmw_zenoh/issues/576>`_) (`#579 <https://github.com/ros2/rmw_zenoh/issues/579>`_)
* fix: use `std::unique_lock` to unlock correctly on Windows (`#570 <https://github.com/ros2/rmw_zenoh/issues/570>`_) (`#574 <https://github.com/ros2/rmw_zenoh/issues/574>`_)
* Switch to std::map for TopicTypeMap (backport `#546 <https://github.com/ros2/rmw_zenoh/issues/546>`_) (`#566 <https://github.com/ros2/rmw_zenoh/issues/566>`_)
* feat: support zenoh config override (backport `#551 <https://github.com/ros2/rmw_zenoh/issues/551>`_) (`#560 <https://github.com/ros2/rmw_zenoh/issues/560>`_)
* Enable Zenoh UDP transport (`#486 <https://github.com/ros2/rmw_zenoh/issues/486>`_) (`#489 <https://github.com/ros2/rmw_zenoh/issues/489>`_)
* feat: introduce the advanced publisher and subscriber (backport `#368 <https://github.com/ros2/rmw_zenoh/issues/368>`_) (`#470 <https://github.com/ros2/rmw_zenoh/issues/470>`_)
* Backport `#294 <https://github.com/ros2/rmw_zenoh/issues/294>`_ to humble  (backport `#471 <https://github.com/ros2/rmw_zenoh/issues/471>`_) (`#472 <https://github.com/ros2/rmw_zenoh/issues/472>`_)
* Align the config with the latest Zenoh. (`#556 <https://github.com/ros2/rmw_zenoh/issues/556>`_) (`#558 <https://github.com/ros2/rmw_zenoh/issues/558>`_)
* Bump zenoh-cpp to 2a127bb, zenoh-c to 3540a3c, and zenoh to f735bf5 (`#503 <https://github.com/ros2/rmw_zenoh/issues/503>`_) (`#512 <https://github.com/ros2/rmw_zenoh/issues/512>`_)
* Added documentation note in the code (`#540 <https://github.com/ros2/rmw_zenoh/issues/540>`_) (`#542 <https://github.com/ros2/rmw_zenoh/issues/542>`_)
* fix: unlock the mutex before making get (`#537 <https://github.com/ros2/rmw_zenoh/issues/537>`_) (`#538 <https://github.com/ros2/rmw_zenoh/issues/538>`_)
* Take wait_set_lock before condition_variable notification for subscriptions (`#528 <https://github.com/ros2/rmw_zenoh/issues/528>`_) (`#535 <https://github.com/ros2/rmw_zenoh/issues/535>`_)
* Switch default durability to volatile (`#521 <https://github.com/ros2/rmw_zenoh/issues/521>`_) (`#531 <https://github.com/ros2/rmw_zenoh/issues/531>`_)
* Honor ignore_local_publications in subscription options (backport `#508 <https://github.com/ros2/rmw_zenoh/issues/508>`_) (`#514 <https://github.com/ros2/rmw_zenoh/issues/514>`_)
* Fixed windows warning (`#500 <https://github.com/ros2/rmw_zenoh/issues/500>`_) (`#519 <https://github.com/ros2/rmw_zenoh/issues/519>`_)
* Config: tune some values for ROS use case, especially with large number of Nodes (>200) (`#509 <https://github.com/ros2/rmw_zenoh/issues/509>`_) (`#516 <https://github.com/ros2/rmw_zenoh/issues/516>`_)
* Fix calculation of current_count_change when event status is updated (`#504 <https://github.com/ros2/rmw_zenoh/issues/504>`_) (`#507 <https://github.com/ros2/rmw_zenoh/issues/507>`_)
* fix: use the default destructor that automatically drops the zenoh reply/query and hence sends the final signal (`#473 <https://github.com/ros2/rmw_zenoh/issues/473>`_) (`#475 <https://github.com/ros2/rmw_zenoh/issues/475>`_)
* Switch to debug log if topic_name not in topic_map (`#454 <https://github.com/ros2/rmw_zenoh/issues/454>`_) (`#465 <https://github.com/ros2/rmw_zenoh/issues/465>`_)
* Bump Zenoh to commit id 3bbf6af (1.2.1 + few commits) (`#456 <https://github.com/ros2/rmw_zenoh/issues/456>`_) (`#462 <https://github.com/ros2/rmw_zenoh/issues/462>`_)
* Contributors: Alejandro Hernández Cordero, ChenYing Kuo (CY), Chris Lalancette, Christophe Bedard, G.A. vd. Hoorn, Julien Enoch, Luca Cominardi, Øystein Sture, Paul Bouchier, Yadunund, yellowhatter, Yuyuan Yuan

0.1.1 (2025-02-04)
------------------
* Bump Zenoh to commit id e4ea6f0 (1.2.0 + few commits) (`#449 <https://github.com/ros2/rmw_zenoh/issues/449>`_)
* Inform users that peers will not discover and communicate with one another until the router is started (`#445 <https://github.com/ros2/rmw_zenoh/issues/445>`_)
* Clear the error after rmw_serialized_message_resize() (`#437 <https://github.com/ros2/rmw_zenoh/issues/437>`_)
* Fix `ZENOH_ROUTER_CHECK_ATTEMPTS` which was not respected (`#429 <https://github.com/ros2/rmw_zenoh/issues/429>`_)
* fix: use the default destructor to drop the member `Payload` (`#421 <https://github.com/ros2/rmw_zenoh/issues/421>`_)
* Remove `gid_hash\_` from `AttachmentData` (`#418 <https://github.com/ros2/rmw_zenoh/issues/418>`_)
* Sync the config with the default config in Zenoh. (`#414 <https://github.com/ros2/rmw_zenoh/issues/414>`_)
* fix: check the context validity before accessing the session (`#407 <https://github.com/ros2/rmw_zenoh/issues/407>`_)
* Fix wan't typo (`#402 <https://github.com/ros2/rmw_zenoh/issues/402>`_)
* Contributors: Alejandro Hernández Cordero, ChenYing Kuo (CY), Chris Lalancette, Julien Enoch, Mahmo Clephas, Yadunud, Mahmoud Mazouz, Timund, Yuyuan Yuan

0.1.0 (2025-01-02)
------------------
* An alternative middleware for ROS 2 based on Zenoh.
* Contributors: Alejandro Hernández Cordero, Alex Day, Bernd Pfrommer, ChenYing Kuo (CY), Chris Lalancette, Christophe Bedard, CihatAltiparmak, Esteve Fernandez, Franco Cipollone, Geoffrey Biggs, Hans-Martin, James Mount, Julien Enoch, Morgan Quigley, Nate Koenig, Shivang Vijay, Yadunund, Yuyuan Yuan, methylDragon
