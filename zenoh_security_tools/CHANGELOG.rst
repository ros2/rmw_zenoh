^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package zenoh_security_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.8 (2025-10-05)
------------------
* Fix commands in zenoh_security_tools README (`#816 <https://github.com/ros2/rmw_zenoh/issues/816>`_)
* Revert "fix: handle missing enclaves_dir argument for zenoh_security_tools" (`#807 <https://github.com/ros2/rmw_zenoh/issues/807>`_)
* Correct a description error in the zenoh_security_tools README (`#794 <https://github.com/ros2/rmw_zenoh/issues/794>`_)
* fix: handle missing enclaves_dir argument for zenoh_security_tools (`#791 <https://github.com/ros2/rmw_zenoh/issues/791>`_)
* Contributors: Barry Xu, Christophe Bedard, Yadunund
i
0.2.7 (2025-09-10)
------------------
* SROS: add ACL rules for TRANSIENT_LOCAL pub/sub (fix `#753 <https://github.com/ros2/rmw_zenoh/issues/753>`_) (`#781 <https://github.com/ros2/rmw_zenoh/issues/781>`_)
* Fix handling of enclave path in zenoh_security_tools (`#772 <https://github.com/ros2/rmw_zenoh/issues/772>`_)
* Contributors: Julien Enoch, Yadunund

0.2.6 (2025-08-21)
------------------

0.2.5 (2025-06-18)
------------------
* Add zenoh_security_tools (`#661 <https://github.com/ros2/rmw_zenoh/issues/661>`_)
* Contributors: Alejandro Hernández Cordero

0.2.4 (2025-04-20)
------------------
* Revert "Add zenoh_security_tools (`#595 <https://github.com/ros2/rmw_zenoh/issues/595>`_) (`#609 <https://github.com/ros2/rmw_zenoh/issues/609>`_)" (`#611 <https://github.com/ros2/rmw_zenoh/issues/611>`_)
  This reverts commit aa23ecd91ee455bc6a529fb3cde0b77a8ba39911.
* Add zenoh_security_tools (`#595 <https://github.com/ros2/rmw_zenoh/issues/595>`_) (`#609 <https://github.com/ros2/rmw_zenoh/issues/609>`_)
  * Added zenoh_security_configuration_tools package written in cpp
  * Added domain id
  * Update policy_parser to use nlohmann_json.hpp
  Since we have the dependency now via `#583 <https://github.com/ros2/rmw_zenoh/issues/583>`_, this is a potential
  improvement to the current string concatenation.
  * Further json changes
  * Refactor package to zenoh_security_tools
  * Inject certificates if enclaves provided
  * Replace existing endpoints with tls instead of hardcoded changes
  * Update README.md
  * Also generate router config with security
  * Drop CLI11 dependency
  ---------
  Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
  Co-authored-by: Michael Carroll <mjcarroll@intrinsic.ai>
  (cherry picked from commit 1dca3c35ba4616827db18c8a4658c259fa982144)
  Co-authored-by: yadunund <yadunund@gmail.com>
* Contributors: mergify[bot], yadunund

0.2.3 (2025-03-20)
------------------

0.2.2 (2025-02-19)
------------------

0.2.1 (2025-02-04)
------------------

0.2.0 (2025-01-02)
------------------
