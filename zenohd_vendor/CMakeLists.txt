cmake_minimum_required(VERSION 3.16)
project(zenohd_vendor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)

include(ExternalProject)

ExternalProject_Add(
    zenoh
    PREFIX ${CMAKE_BINARY_DIR}/_cargo_deps
    GIT_REPOSITORY https://github.com/eclipse-zenoh/zenoh.git
    GIT_TAG 0.10.1-rc
    BUILD_COMMAND cargo build --package zenohd --release
    CONFIGURE_COMMAND "" # Disable the configure step
    INSTALL_COMMAND ""
    BUILD_IN_SOURCE 1
)

# Install the zenohd executable from the rust crate
install(
    PROGRAMS ${CMAKE_BINARY_DIR}/_cargo_deps/src/zenoh/target/release/zenohd
    DESTINATION bin
)

# TODO(francocipollone): Install zenoh router plugins

ament_package()
