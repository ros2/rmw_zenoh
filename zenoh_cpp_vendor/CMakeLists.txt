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

# Set VCS_VERSION to 1.6.2 commits of zenoh/zenoh-c/zenoh-cpp to benefit from:
#  * Various performance improvements:
#    - https://github.com/eclipse-zenoh/zenoh/pull/2109
#    - https://github.com/eclipse-zenoh/zenoh/pull/2127
#    - https://github.com/eclipse-zenoh/zenoh/pull/2174
#  * Shared Memory improvements:
#    - https://github.com/eclipse-zenoh/zenoh/pull/2117
#    - https://github.com/eclipse-zenoh/zenoh/pull/2136
#    - https://github.com/eclipse-zenoh/zenoh-c/pull/1105
#    - https://github.com/eclipse-zenoh/zenoh-c/pull/1106
#    - https://github.com/eclipse-zenoh/zenoh/pull/2193
#  * Fix IPv6 address parsing for TLS and QUIC locators:
#    - https://github.com/eclipse-zenoh/zenoh/pull/2138
#  * Fix a bug leading to 100% CPU usage in certain circumstances :
#    - https://github.com/eclipse-zenoh/zenoh/pull/2140
#  * Fix zenoh/stats feature support
#    - https://github.com/eclipse-zenoh/zenoh-c/pull/1110
#  * Fix partial reconnection in case of multilink configuration
#    - https://github.com/eclipse-zenoh/zenoh/pull/2173
#  * Fix Access Control subject matching
#    - https://github.com/eclipse-zenoh/zenoh/pull/2141
# * Fix 1.6.1 warnings: "Request to update QueryableInfo for inexistent face id"
#   - https://github.com/eclipse-zenoh/zenoh/pull/2205
# * Precommit SHM pages, avoiding overcommit and possible crash if SHM exhausts
#   - https://github.com/eclipse-zenoh/zenoh/pull/2175
ament_vendor(zenoh_c_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
  VCS_VERSION f376456ccf75ed837a21a186bdf5191cba50eb3b
  CMAKE_ARGS
    "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
    "-DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE"
    "-DZENOHC_CUSTOM_TARGET=${ZENOHC_CUSTOM_TARGET}"
  PATCHES ${CMAKE_CURRENT_SOURCE_DIR}/pin-rust-1.75.0.patch
)

ament_export_dependencies(zenohc)

ament_vendor(zenoh_cpp_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-cpp
  VCS_VERSION 0cd54f291039a65b96921a5951a66aeef088e67c
  CMAKE_ARGS
    -DZENOHCXX_ZENOHC=OFF
)

externalproject_add_stepdependencies(zenoh_cpp_vendor configure zenoh_c_vendor)

ament_export_dependencies(zenohcxx)

ament_package()
