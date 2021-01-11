cmake_minimum_required(VERSION 3.9)

project(microcdr_vendor)

find_package(ament_cmake REQUIRED)

macro(build_microcdr)
  set(microcdr_REV "v1.2.0")
  set(extra_cmake_args)

  get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if(NOT multi_config AND DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  include(ExternalProject)
  externalproject_add(microcdr-${microcdr_REV}
    GIT_REPOSITORY https://github.com/eProsima/Micro-CDR.git
    GIT_TAG ${microcdr_REV}
    GIT_SHALLOW OFF
    TIMEOUT 60
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/microcdr_install
      ${extra_cmake_args}
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/microcdr_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
  )
endmacro()

option(BUILTIN_microcdr "Force built-in version of microcdr" OFF)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")

find_package(microcdr QUIET)

if(NOT microcdr_FOUND OR BUILTIN_microcdr)
  build_microcdr()
endif()

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS microcdr_vendor-extras.cmake)
