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

  include(get_cargo_version.cmake)
  get_cargo_version()

  # By default, use zenoh-c 1.8.0
  set(zenoh_c_commit c1a477d3f5efc3abc0f0cf95e0773225d00dea3b)
  if(CARGO_VERSION VERSION_GREATER_EQUAL "1.75" AND CARGO_VERSION VERSION_LESS "1.76")
    # Use ROS/rust-1.75 branch of zenoh-c
    set(zenoh_c_commit 0e3ec084142382096d09f7fc2ad59385998b92e6)
  endif()

  ament_vendor(zenoh_c_vendor
    VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
    VCS_VERSION ${zenoh_c_commit}
    CMAKE_ARGS
      "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
      "-DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE"
      "-DZENOHC_CUSTOM_TARGET=${ZENOHC_CUSTOM_TARGET}"
      ${FLAGS_FOR_CARGO_VERSION}
  )

  ament_vendor(zenoh_cpp_vendor
    VCS_URL https://github.com/eclipse-zenoh/zenoh-cpp
    VCS_VERSION 776c59651a61dc53689dd68beace147e67a83a27
    CMAKE_ARGS
      -DZENOHCXX_ZENOHC=OFF
  )

  externalproject_add_stepdependencies(zenoh_cpp_vendor configure zenoh_c_vendor)
endif()

ament_export_dependencies(zenohc)
ament_export_dependencies(zenohcxx)

ament_package()
