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

set(ZENOH_C_TAG "0.5.0-b5")

include(ExternalProject)
ExternalProject_Add(zenoh-c-${ZENOH_C_TAG}
  PREFIX zenoh-c
  GIT_REPOSITORY https://github.com/eclipse-zenoh/zenoh-c.git
  GIT_TAG ${ZENOH_C_TAG}
  GIT_SHALLOW FALSE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND BUILD_TYPE=${CARGO_BUILD_FLAG} make
  BUILD_IN_SOURCE TRUE
  INSTALL_COMMAND PREFIX=${CMAKE_INSTALL_PREFIX} make install
  )

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS "zenoh_vendor-extras.cmake")
