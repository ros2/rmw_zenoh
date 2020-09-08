cmake_minimum_required(VERSION 3.5)
project(zenoh_vendor VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

set(extra_cmake_args)

if(DEFINED CMAKE_TOOLCHAIN_FILE)
  list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CARGO_BUILD_FLAG "")
  set(ZENOH_OBJECTS_DIR "debug")
else()
  set(CARGO_BUILD_FLAG "--release")
  set(ZENOH_OBJECTS_DIR "release")
endif()

include(ExternalProject)
ExternalProject_Add(zenoh-20200825
  PREFIX zenoh
  GIT_REPOSITORY https://github.com/eclipse-zenoh/zenoh.git
  GIT_TAG 2ac16d08b604604594f3627ff24e5613238af3e0
  GIT_SHALLOW FALSE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND cargo build ${CARGO_BUILD_FLAG} -p zenoh-ffi
  BUILD_IN_SOURCE TRUE
  INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_INSTALL_PREFIX}/lib"
  COMMAND ${CMAKE_COMMAND} -E copy
      "${CMAKE_CURRENT_BINARY_DIR}/zenoh/src/zenoh-20200825/target/${ZENOH_OBJECTS_DIR}/libzenoh_ffi.so"
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
