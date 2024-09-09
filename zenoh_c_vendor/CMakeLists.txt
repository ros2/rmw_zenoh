cmake_minimum_required(VERSION 3.16)
project(zenoh_c_vendor)

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
set(ZENOHC_CARGO_FLAGS "--no-default-features$<SEMICOLON>--features=shared-memory zenoh/transport_compression zenoh/transport_tcp zenoh/transport_tls")

# Set VCS_VERSION to include latest changes from zenoh-c to benefit from :
# - https://github.com/eclipse-zenoh/zenoh-c/pull/340 (fix build issue)
# - https://github.com/eclipse-zenoh/zenoh/pull/1021 (fix timeout issue with queries)
# - https://github.com/eclipse-zenoh/zenoh/pull/1022 (fix empty messages received if payload >btach size)
# - https://github.com/eclipse-zenoh/zenoh-c/pull/358 (fix debian packaging issue: https://github.com/jspricke/ros-deb-builder-action/issues/49)
# - https://github.com/eclipse-zenoh/zenoh/pull/1150 (fix deadlock issue https://github.com/ros2/rmw_zenoh/issues/182)
# - https://github.com/eclipse-zenoh/zenoh-c/pull/620 (fix ze_querying_subscriber_get API to query newly discovered publishers)
ament_vendor(zenoh_c_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
  VCS_VERSION 8d5b11ce114a4eb591f15891878badeb53e56c48
  CMAKE_ARGS
    "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
    "-DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE"
    "-DZENOHC_CUSTOM_TARGET=${ZENOHC_CUSTOM_TARGET}"
)

ament_export_dependencies(zenohc)

ament_package()
