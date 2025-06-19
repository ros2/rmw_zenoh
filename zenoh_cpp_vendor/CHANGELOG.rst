^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package zenoh_cpp_vendor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2025-06-19)
------------------
* Bump Zenoh to 1.4.0 (`#652 <https://github.com/ros2/rmw_zenoh/issues/652>`_) (`#659 <https://github.com/ros2/rmw_zenoh/issues/659>`_)
  (cherry picked from commit 4d066979d3cc1b1cf5ed48a806aa12206d0aab7f)
  Co-authored-by: Julien Enoch <julien.e@zettascale.tech>
* fix: pin rust toolchain to v1.75.0 (`#602 <https://github.com/ros2/rmw_zenoh/issues/602>`_) (`#636 <https://github.com/ros2/rmw_zenoh/issues/636>`_)
  * fix: pin rust toolchain to v1.75.0
  * Apply the suggested change
  ---------
  (cherry picked from commit 33b250e4541718977f35ea6d518a96f5af7465fd)
  Co-authored-by: Yuyuan Yuan <az6980522@gmail.com>
  Co-authored-by: yadunund <yadunund@gmail.com>
* fix: use the right commit to bump zenoh to v1.3.2 (`#607 <https://github.com/ros2/rmw_zenoh/issues/607>`_) (`#633 <https://github.com/ros2/rmw_zenoh/issues/633>`_)
  (cherry picked from commit 181c27de53da3f752dded1e6cd61d1ab2c3b19ab)
  Co-authored-by: Yuyuan Yuan <az6980522@gmail.com>
  Co-authored-by: Yadunund <yadunund@gmail.com>
* Bump Zenoh to v1.3.2 and improve e2e reliability with HeartbeatSporadic (`#591 <https://github.com/ros2/rmw_zenoh/issues/591>`_) (`#594 <https://github.com/ros2/rmw_zenoh/issues/594>`_)
  * Bump zenoh-c to ffa4bdd, zenoh-cpp to 868fdad and zenoh to 3f62ebc
  * Apply same config changes than `eclipse-zenoh/zenoh#1825 <https://github.com/eclipse-zenoh/zenoh/issues/1825>`_
  * Apply same config changes than `eclipse-zenoh/zenoh#1850 <https://github.com/eclipse-zenoh/zenoh/issues/1850>`_
  * For RELIABLE+TRANSIENT_LOCAL topics, enable sample_miss_detection and recovery for end-to-end reliability
  (cherry picked from commit a9ab960d57e1bb5eaa966e3dd90a5b32b5a14b61)
  Co-authored-by: Julien Enoch <julien.e@zettascale.tech>
* build(deps): bump zenoh-cpp from 2a127bb to 8ad67f6, zenoh-c from 3540a3c to e6a1971, and zenoh from f735bf5 to b661454 (`#544 <https://github.com/ros2/rmw_zenoh/issues/544>`_) (`#550 <https://github.com/ros2/rmw_zenoh/issues/550>`_)
  Co-authored-by: Yuyuan Yuan <az6980522@gmail.com>
* Enable Zenoh UDP transport (`#486 <https://github.com/ros2/rmw_zenoh/issues/486>`_) (`#489 <https://github.com/ros2/rmw_zenoh/issues/489>`_)
  Co-authored-by: Hugal31 <hugo.laloge@gmail.com>
* Bump zenoh-cpp to 2a127bb, zenoh-c to 3540a3c, and zenoh to f735bf5 (`#503 <https://github.com/ros2/rmw_zenoh/issues/503>`_) (`#512 <https://github.com/ros2/rmw_zenoh/issues/512>`_)
  (cherry picked from commit 19b7f7767917c1168ba081782ecde6b39d26262e)
  Co-authored-by: Luca Cominardi <luca.cominardi@gmail.com>
* Bump zenoh-c to 261493 and zenoh-cpp to 5dfb68c (`#463 <https://github.com/ros2/rmw_zenoh/issues/463>`_) (`#467 <https://github.com/ros2/rmw_zenoh/issues/467>`_)
  (cherry picked from commit baa4e875eca13dc16cebc61842cd6cf2600158fb)
  Co-authored-by: Julien Enoch <julien.e@zettascale.tech>
* Bump Zenoh to commit id 3bbf6af (1.2.1 + few commits) (`#456 <https://github.com/ros2/rmw_zenoh/issues/456>`_) (`#462 <https://github.com/ros2/rmw_zenoh/issues/462>`_)
  * Bump Zenoh to 3bbf6af
  * Config: add new autoconnect_strategy setting
  (cherry picked from commit 1601a69622cd913d0f34e64f1840f3b159ca5e75)
  Co-authored-by: Julien Enoch <julien.e@zettascale.tech>
* Contributors: mergify[bot]

0.1.1 (2025-02-04)
------------------
* Bump Zenoh to commit id e4ea6f0 (1.2.0 + few commits) (`#449 <https://github.com/ros2/rmw_zenoh/issues/449>`_)
* Bump zenoh-c and zenoh-cpp to 1.1.1 (`#430 <https://github.com/ros2/rmw_zenoh/issues/430>`_)
* Update Zenoh version (`#410 <https://github.com/ros2/rmw_zenoh/issues/410>`_)
* Contributors: ChenYing Kuo (CY), Julien Enoch, Yadunund, Yuyuan Yuan

0.1.0 (2025-01-02)
------------------
* Vendors zenoh-cpp for rmw_zenoh.
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Franco Cipollone, Julien Enoch, Yadunund, Yuyuan Yuan
