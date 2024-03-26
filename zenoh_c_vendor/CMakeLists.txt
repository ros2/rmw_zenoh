cmake_minimum_required(VERSION 3.16)
project(zenoh_c_vendor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# Disable default features and only enable tcp transport for zenoh. This reduces
# build time but more importantly allows us to limit the number of threads that
# the zenoh session can spin via the session config as well as the ASYNC_STD_THREAD_COUNT
# environment variable. Without this limit, applications with multiple zenoh sessions can
# encounter system resource errors when trying to create new threads.
# Once zenoh migrates to relying on tokio for its async runtime,
# see https://github.com/eclipse-zenoh/zenoh/pull/566, we can consider
# removing these flags since with tokio, zenoh can better manage the threads it spins
# with the help of thread pools.
# Note: We separate the two args needed for cargo with "$<SEMICOLON>" and not ";" as the
# latter is a list separater in cmake and hence the string will be split into two
# when expanded.
set(ZENOHC_CARGO_FLAGS "--no-default-features$<SEMICOLON>--features=zenoh/transport_tcp zenoh/shared-memory")

# Always force rustc 1.75.0.
# This is the default version of rustc available via apt in Ubuntu Noble.
# We explictly use the +1.75.0 toolchain when building on Ubuntu Jammy.
set(ZENOHC_CARGO_CHANNEL "1.75.0")

# Apply the patch to the zenoh-c CMakeLists.txt only if building an ROS 2 version
# newer than Iron. The patch removes the ability to set ZENOHC_CARGO_CHANNEL
# via cmake args
set(pathces "")
string(SUBSTRING "$ENV{ROS_DISTRO}" 0 1 first_char)
if("${first_char}" STRGREATER "i")
  set(patches "patches")
endif()

# Set VCS_VERSION to include changes from https://github.com/eclipse-zenoh/zenoh/pull/802
# which was synced to zenoh-c in https://github.com/eclipse-zenoh/zenoh-c/pull/272.
ament_vendor(zenoh_c_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
  VCS_VERSION 10176b911096cb92b8ee46bc491b78079ee26c20
  CMAKE_ARGS
    "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
    "-DZENOHC_CARGO_CHANNEL=${ZENOHC_CARGO_CHANNEL}"
  PATCHES patches
)

# set(INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-prefix/install")
# install(
#   DIRECTORY "${INSTALL_DIR}/lib/"
#   DESTINATION "lib/${PROJECT_NAME}"
#   USE_SOURCE_PERMISSIONS
# )

ament_package()
