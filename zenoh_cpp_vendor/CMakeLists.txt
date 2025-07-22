cmake_minimum_required(VERSION 3.16)
project(zenoh_cpp_vendor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# Add a set of usefull features that are not enabled by default in zenoh-c.
# For a complete list of features see: https://github.com/eclipse-zenoh/zenoh/blob/main/zenoh/Cargo.toml
# Note: the list of features are separated with whitespaces.
# However, if you want to add other cargo build flags, use "$<SEMICOLON>" as separator and not ";" as the
# latter is a list separater in cmake and hence the string will be split into two when expanded.
set(ZENOHC_CARGO_FLAGS "--features=shared-memory zenoh/transport_serial")

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
  VCS_VERSION ce429e221c46932a79f3377d99a49e4d64c5e25e
  CMAKE_ARGS
    "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
    "-DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE"
    "-DZENOHC_CUSTOM_TARGET=${ZENOHC_CUSTOM_TARGET}"
  PATCHES ${CMAKE_CURRENT_SOURCE_DIR}/pin-rust-1.75.0.patch
)

ament_export_dependencies(zenohc)

ament_vendor(zenoh_cpp_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-cpp
  VCS_VERSION 2f648c6be51adffb99537134f2b60a7f8b30e490
  CMAKE_ARGS
    -DZENOHCXX_ZENOHC=OFF
)

externalproject_add_stepdependencies(zenoh_cpp_vendor configure zenoh_c_vendor)

ament_export_dependencies(zenohcxx)

ament_package()
