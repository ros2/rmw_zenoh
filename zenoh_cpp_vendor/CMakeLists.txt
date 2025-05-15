cmake_minimum_required(VERSION 3.16)
project(zenoh_cpp_vendor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# Disable default features and enable only the most useful ones. This reduces build time and footprint.
# For a complete list of features see: https://github.com/eclipse-zenoh/zenoh/blob/main/zenoh/Cargo.toml
# Note: We separate the two args needed for cargo with "$<SEMICOLON>" and not ";" as the
# latter is a list separater in cmake and hence the string will be split into two
# when expanded.
set(ZENOHC_CARGO_FLAGS "--no-default-features$<SEMICOLON>--features=shared-memory zenoh/transport_compression zenoh/transport_tcp zenoh/transport_udp zenoh/transport_tls")

# Set VCS_VERSION to include latest changes from zenoh/zenoh-c/zenoh-cpp to benefit from:
# - Fix a bug leading to invalid inapropriate "Unable to push non droppable network message" log and transport closure:
#    - https://github.com/eclipse-zenoh/zenoh/pull/1855
# - Fix crash with highly chunked keys:
#    - https://github.com/eclipse-zenoh/zenoh/pull/1826
# - Resolve issue with closing the Session in atexit:
#    - https://github.com/eclipse-zenoh/zenoh/pull/1632
# - Change `Session::close()` implementation so it can be safely waited and awaited in `atexit``
#    - https://github.com/eclipse-zenoh/zenoh/pull/1632
# - Add QoS overwrite interceptor allowing for instance a Router to be configured to change QoS on the fly
#    - https://github.com/eclipse-zenoh/zenoh/pull/1825
# - Add link protocols as subject to interceptors (access_control, downsampling or qos overwrite):
#    - https://github.com/eclipse-zenoh/zenoh/pull/1850
# - Add new non periodic last sample miss detection mechanism for Advanced Publisher:
#    - https://github.com/eclipse-zenoh/zenoh/pull/1861
# - Improve tracing for better analysis on the system like rmw_zenoh
#    - https://github.com/eclipse-zenoh/zenoh/pull/1844
ament_vendor(zenoh_c_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
  VCS_VERSION f60bbaba51417256b267f59f909a48eb99f1833b
  CMAKE_ARGS
    "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
    "-DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE"
    "-DZENOHC_CUSTOM_TARGET=${ZENOHC_CUSTOM_TARGET}"
  PATCHES ${CMAKE_CURRENT_SOURCE_DIR}/pin-rust-1.75.0.patch
)

ament_export_dependencies(zenohc)

ament_vendor(zenoh_cpp_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-cpp
  VCS_VERSION 868fdad0e7418e8f8cb96e94c89a3aed05905e63
  CMAKE_ARGS
    -DZENOHCXX_ZENOHC=OFF
)

externalproject_add_stepdependencies(zenoh_cpp_vendor configure zenoh_c_vendor)

ament_export_dependencies(zenohcxx)

ament_package()
