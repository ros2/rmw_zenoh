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

  # zenoh version:     https://github.com/eclipse-zenoh/zenoh/commit/9fcd9cb5d364192c3e8a27e66de76f4bc750d1d5
  # zenoh-c version:   https://github.com/eclipse-zenoh/zenoh-c/commit/07b0d432121933cb368528153df000463292635d
  # zenoh-cpp version: https://github.com/eclipse-zenoh/zenoh-cpp/commit/1e343e61b92a73af3bc247c178214bcc1042c34c
  # This corresponds to zenoh-c 1.10.1 plus few fixes, including https://github.com/eclipse-zenoh/zenoh/pull/2779
  # and few changes to build with Rust 1.75.
  set(zenoh_c_commit 07b0d432121933cb368528153df000463292635d)
  set(zenoh_cpp_commit 1e343e61b92a73af3bc247c178214bcc1042c34c)

  ament_vendor(zenoh_c_vendor
    VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
    VCS_VERSION ${zenoh_c_commit}
    CMAKE_ARGS
      "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
      "-DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE"
      "-DZENOHC_CUSTOM_TARGET=${ZENOHC_CUSTOM_TARGET}"
  )

  ament_vendor(zenoh_cpp_vendor
    VCS_URL https://github.com/eclipse-zenoh/zenoh-cpp
<<<<<<< HEAD
    VCS_VERSION af381b420cc8837ac7da42c9984594ef8f110e90
=======
    VCS_VERSION ${zenoh_cpp_commit}
>>>>>>> 1f7c62a (Bump zenoh to 1.10.1 + extra fixes (commit 9fcd9cb) (#1068))
    CMAKE_ARGS
      -DZENOHCXX_ZENOHC=OFF
  )

  externalproject_add_stepdependencies(zenoh_cpp_vendor configure zenoh_c_vendor)
endif()

ament_export_dependencies(zenohc)
ament_export_dependencies(zenohcxx)

ament_package()
