cmake_minimum_required(VERSION 3.16)
project(zenoh_c_vendor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# Disable default features and only enable tcp, tls and shared-memory transport for zenoh.
# This reduces build time but more importantly allows us to limit the number of threads that
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
set(ZENOHC_CARGO_FLAGS "--no-default-features$<SEMICOLON>--features=zenoh/transport_tcp zenoh/transport_compression zenoh/transport_tls zenoh/shared-memory")

# Set VCS_VERSION to include latest changes from zenoh-c to benefit from :
# - https://github.com/eclipse-zenoh/zenoh-c/pull/340 (fix build issue)
# - https://github.com/eclipse-zenoh/zenoh/pull/1021 (fix timeout issue with queries)
# - https://github.com/eclipse-zenoh/zenoh/pull/1022 (fix empty messages received if payload >btach size)
# - https://github.com/eclipse-zenoh/zenoh-c/pull/358 (fix debian packaging issue: https://github.com/jspricke/ros-deb-builder-action/issues/49)
ament_vendor(zenoh_c_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
  VCS_VERSION 390ec14e1b3785cd771bd6931db65062222bb735
  CMAKE_ARGS
  "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
)
# set(INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-prefix/install")
# install(
# DIRECTORY "${INSTALL_DIR}/lib/"
# DESTINATION "lib/${PROJECT_NAME}"
# USE_SOURCE_PERMISSIONS
# )
ament_package()
