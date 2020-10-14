cmake_minimum_required(VERSION 3.5)
project(zenoh_vendor VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

set(extra_cmake_args)

if(DEFINED CMAKE_TOOLCHAIN_FILE)
  list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CARGO_BUILD_FLAG "Debug")
  set(ZENOH_OBJECTS_DIR "debug")
else()
  set(CARGO_BUILD_FLAG "--release")
  set(ZENOH_OBJECTS_DIR "release")
endif()

include(ExternalProject)
ExternalProject_Add(zenoh-c-20201006
  PREFIX zenoh-c
  GIT_REPOSITORY https://github.com/eclipse-zenoh/zenoh-c.git
  GIT_TAG 6e5f5c89ff3b6791b74bf215245f170da210f5fc
  GIT_SHALLOW FALSE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND BUILD_TYPE=${CARGO_BUILD_FLAG} make
  BUILD_IN_SOURCE TRUE
  INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_INSTALL_PREFIX}/lib"
  COMMAND ${CMAKE_COMMAND} -E copy
      "${CMAKE_CURRENT_BINARY_DIR}/zenoh-c/src/zenoh-c-20201006/target/${ZENOH_OBJECTS_DIR}/libzenohc.so"
      "${CMAKE_INSTALL_PREFIX}/lib"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_INSTALL_PREFIX}/include/zenoh"
  COMMAND ${CMAKE_COMMAND} -E copy
      "${CMAKE_CURRENT_BINARY_DIR}/zenoh-c/src/zenoh-c-20201006/include/zenoh.h"
      "${CMAKE_INSTALL_PREFIX}/include/zenoh"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_INSTALL_PREFIX}/include/zenoh/zenoh"
  COMMAND ${CMAKE_COMMAND} -E copy
      "${CMAKE_CURRENT_BINARY_DIR}/zenoh-c/src/zenoh-c-20201006/include/zenoh/net.h"
      "${CMAKE_INSTALL_PREFIX}/include/zenoh/zenoh"
  )

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS "zenoh_vendor-extras.cmake")
