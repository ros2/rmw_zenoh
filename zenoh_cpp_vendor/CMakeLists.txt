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

# Set VCS_VERSION to 1.7.1 commits of zenoh/zenoh-c/zenoh-cpp to benefit from:
# * Query cancellation
#   - https://github.com/eclipse-zenoh/zenoh/pull/2223
#   - https://github.com/eclipse-zenoh/zenoh-c/pull/1134
#   - https://github.com/eclipse-zenoh/zenoh-cpp/pull/663
# * Access to transport optimization SHM provider
#   - https://github.com/eclipse-zenoh/zenoh/pull/2221
#   - https://github.com/eclipse-zenoh/zenoh-c/pull/1132
#   - https://github.com/eclipse-zenoh/zenoh-cpp/pull/654
# * Metrics per key (requires to add "zenoh/stats" feature to ZENOHC_CARGO_FLAGS for build)
#   - https://github.com/eclipse-zenoh/zenoh/pull/2284
ament_vendor(zenoh_c_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
  VCS_VERSION 73e7a0bdcf78b8263a02b7aee453ee6de8187589
  CMAKE_ARGS
    "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
    "-DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE"
    "-DZENOHC_CUSTOM_TARGET=${ZENOHC_CUSTOM_TARGET}"
  PATCHES ${CMAKE_CURRENT_SOURCE_DIR}/pin-rust-1.75.0.patch
)

ament_export_dependencies(zenohc)

ament_vendor(zenoh_cpp_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-cpp
  VCS_VERSION 47a80d345d74ebef16f3783423bf3bbd89cd6c30
  CMAKE_ARGS
    -DZENOHCXX_ZENOHC=OFF
)

externalproject_add_stepdependencies(zenoh_cpp_vendor configure zenoh_c_vendor)

ament_export_dependencies(zenohcxx)

ament_package()
