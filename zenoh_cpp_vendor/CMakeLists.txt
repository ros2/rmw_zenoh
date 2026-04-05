cmake_minimum_required(VERSION 3.16)
project(zenoh_cpp_vendor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

option(USE_SYSTEM_ZENOH "Use non-vendored zenoh from the system" OFF)

if(USE_SYSTEM_ZENOH)
  find_package(zenohc REQUIRED)
  find_package(zenohcxx REQUIRED)
  message(STATUS "Using system zenohc version: ${zenohc_VERSION}")
  message(STATUS "Using system zenohcxx version: ${zenohcxx_VERSION}")
else()
  # Add a set of usefull features that are not enabled by default in zenoh-c.
  # For a complete list of features see: https://github.com/eclipse-zenoh/zenoh/blob/main/zenoh/Cargo.toml
  # Note: the list of features are separated with whitespaces.
  # However, if you want to add other cargo build flags, use "$<SEMICOLON>" as separator and not ";" as the
  # latter is a list separater in cmake and hence the string will be split into two when expanded.
  set(ZENOHC_CARGO_FLAGS "--features=shared-memory zenoh/transport_serial")

  # Set VCS_VERSION to 1.8.0 commits of zenoh/zenoh-c/zenoh-cpp to benefit from:
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
  # * Add support for TOML configuration files:
  #   - https://github.com/eclipse-zenoh/zenoh/pull/2364
  # * Fix handling wrapping of sequence numbers in advanced pub/sub
  #   - https://github.com/eclipse-zenoh/zenoh/pull/2318
  # * Fix bugs in gossip scouting:
  #   - https://github.com/eclipse-zenoh/zenoh/pull/2347
  # * Fix liveliness token memory leak:
  #   - https://github.com/eclipse-zenoh/zenoh/pull/2357
  # * Fixes for TLS:
  #   - https://github.com/eclipse-zenoh/zenoh/pull/2401
  #   - https://github.com/eclipse-zenoh/zenoh/pull/2432
  ament_vendor(zenoh_c_vendor
    VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
    VCS_VERSION 7b8478618861972b9f0115da75e694c9f0a2af5a
    CMAKE_ARGS
      "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
      "-DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE"
      "-DZENOHC_CUSTOM_TARGET=${ZENOHC_CUSTOM_TARGET}"
      "-DZENOHC_MSRV_1_75=TRUE"
  )

  ament_vendor(zenoh_cpp_vendor
    VCS_URL https://github.com/eclipse-zenoh/zenoh-cpp
    VCS_VERSION c3f98b7fad41bd91b7eb845d7853faadaf2249a5
    CMAKE_ARGS
      -DZENOHCXX_ZENOHC=OFF
  )

  externalproject_add_stepdependencies(zenoh_cpp_vendor configure zenoh_c_vendor)
endif()

ament_export_dependencies(zenohc)
ament_export_dependencies(zenohcxx)

ament_package()
