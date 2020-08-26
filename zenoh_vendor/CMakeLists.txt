cmake_minimum_required(VERSION 3.5)
project(zenoh_vendor VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

set(extra_cmake_args)

if(DEFINED CMAKE_TOOLCHAIN_FILE)
  list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
endif()

include(ExternalProject)
ExternalProject_Add(zenoh-20200825
  PREFIX zenoh
  GIT_REPOSITORY https://github.com/eclipse-zenoh/zenoh.git
  GIT_TAG 2a93c83afb7b0865fb99d5a6fc4b153badc3b8c7
  GIT_SHALLOW TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND cargo build --release -p zenoh-ffi
  BUILD_IN_SOURCE TRUE
  INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_INSTALL_PREFIX}/lib"
  COMMAND ${CMAKE_COMMAND} -E copy
      "${CMAKE_CURRENT_BINARY_DIR}/zenoh/src/zenoh-20200825/target/release/libzenoh_ffi.so"
      "${CMAKE_INSTALL_PREFIX}/lib"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_INSTALL_PREFIX}/include/zenoh"
  COMMAND ${CMAKE_COMMAND} -E copy
      "${CMAKE_CURRENT_BINARY_DIR}/zenoh/src/zenoh-20200825/zenoh-ffi/include/zenoh-ffi.h"
      "${CMAKE_INSTALL_PREFIX}/include/zenoh"
  COMMAND ${CMAKE_COMMAND} -E copy
      "${CMAKE_CURRENT_BINARY_DIR}/zenoh/src/zenoh-20200825/zenoh-ffi/include/zenoh-types.h"
      "${CMAKE_INSTALL_PREFIX}/include/zenoh"
  )

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS "zenoh_vendor-extras.cmake")
