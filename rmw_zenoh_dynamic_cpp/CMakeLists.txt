# Copyright 2020 ADLINK, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(rmw_zenoh_dynamic_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw_zenoh_common_cpp REQUIRED)

find_package(zenoh_pico_vendor REQUIRED)
find_package(zenoh_pico REQUIRED)

find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

include_directories(include)

add_library(rmw_zenoh_dynamic_cpp
  src/get_session.cpp
  src/rmw.cpp
  src/rmw_init.cpp
  src/identifier.cpp
  src/rmw_publisher.cpp
  src/get_publisher.cpp
  src/get_subscriber.cpp
  src/rmw_node.cpp
  src/rmw_subscription.cpp
  src/publisher.cpp
)

# specific order: dependents before dependencies
ament_target_dependencies(rmw_zenoh_dynamic_cpp
  "rcpputils"
  "rcutils"
  "rosidl_typesupport_introspection_c"
  "rosidl_typesupport_introspection_cpp"
  "rmw"
  "rmw_zenoh_common_cpp"
  "rosidl_runtime_c"
  "zenoh_pico_vendor"
  "zenoh_pico"
)

configure_rmw_library(rmw_zenoh_dynamic_cpp)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
PRIVATE "rmw_zenoh_DYNAMIC_CPP_BUILDING_LIBRARY")

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(rmw_zenoh_dynamic_cpp)

ament_export_dependencies(rcpputils)
ament_export_dependencies(rcutils)
ament_export_dependencies(rmw)
ament_export_dependencies(rmw_zenoh_common_cpp)
ament_export_dependencies(rosidl_runtime_c)
ament_export_dependencies(rosidl_typesupport_introspection_c)
ament_export_dependencies(rosidl_typesupport_introspection_cpp)

register_rmw_implementation(
  "c:rosidl_typesupport_c:rosidl_typesupport_introspection_c"
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_introspection_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(osrf_testing_tools_cpp REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_add_gtest(test_get_native_entities
    test/test_get_native_entities.cpp)
  ament_target_dependencies(test_get_native_entities
    osrf_testing_tools_cpp rcutils rmw test_msgs
  )
  target_link_libraries(test_get_native_entities rmw_zenoh_dynamic_cpp)
endif()

ament_package(
  CONFIG_EXTRAS_POST "rmw_zenoh_dynamic_cpp-extras.cmake"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rmw_zenoh_dynamic_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
