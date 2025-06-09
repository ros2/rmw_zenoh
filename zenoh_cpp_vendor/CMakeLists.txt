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

# Set VCS_VERSION to 1.4.0 commits of zenoh/zenoh-c/zenoh-cpp to benefit from:
# - Add a "bind" config option for endpoints:
#    - https://github.com/eclipse-zenoh/zenoh/pull/1892
# - Add a "low_pass_filter" config:
#    - https://github.com/eclipse-zenoh/zenoh/pull/1868
#    - https://github.com/eclipse-zenoh/zenoh/pull/1895
# - Add "zids" config parameter to "qos_overwrite":
#    - https://github.com/eclipse-zenoh/zenoh/pull/1908
# - Add support for link weights:
#    - https://github.com/eclipse-zenoh/zenoh/pull/1914
# - Add support for DiffServ's DSCP config for endpoints:
#    - https://github.com/eclipse-zenoh/zenoh/pull/1937
ament_vendor(zenoh_c_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
  VCS_VERSION 6bea1f1ebc29412548f36af91cf2225c8bf476d4
  CMAKE_ARGS
    "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
    "-DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE"
    "-DZENOHC_CUSTOM_TARGET=${ZENOHC_CUSTOM_TARGET}"
  PATCHES ${CMAKE_CURRENT_SOURCE_DIR}/pin-rust-1.75.0.patch
)

ament_export_dependencies(zenohc)

ament_vendor(zenoh_cpp_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-cpp
  VCS_VERSION 7379592436398079934f4296d2fa90217f8eddf9
  CMAKE_ARGS
    -DZENOHCXX_ZENOHC=OFF
)

externalproject_add_stepdependencies(zenoh_cpp_vendor configure zenoh_c_vendor)

ament_export_dependencies(zenohcxx)

ament_package()
