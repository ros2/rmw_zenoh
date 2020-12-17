cmake_minimum_required(VERSION 3.5)
project(rmw_zenoh_common_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined,error")
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)

find_package(fastrtps_cmake_module REQUIRED)
find_package(fastcdr REQUIRED CONFIG)

find_package(rosidl_generator_c REQUIRED)
find_package(rosidl_typesupport_zenoh_c REQUIRED)
find_package(rosidl_typesupport_zenoh_cpp REQUIRED)

include_directories(include)

add_library(rmw_zenoh_common_cpp
  src/rmw_zenoh_common_event.cpp
  src/rmw_zenoh_common_get_node_info_and_types.cpp
  src/rmw_zenoh_common_get_service_names_and_types.cpp
  src/rmw_zenoh_common_get_topic_names_and_types.cpp
  src/rmw_zenoh_common_get_topic_endpoint_info.cpp
  src/rmw_zenoh_common_graph_info.cpp
  src/rmw_zenoh_common_gids.cpp
  src/rmw_zenoh_common_guard_conditions.cpp
  src/rmw_zenoh_common_init.cpp
  src/rmw_zenoh_common_init_options.cpp
  src/rmw_zenoh_common_logging.cpp
  src/rmw_zenoh_common_node.cpp
  src/rmw_zenoh_common_publish.cpp
  src/rmw_zenoh_common_publisher.cpp
  src/rmw_zenoh_common_serialization.cpp
  src/rmw_zenoh_common_service.cpp
  src/rmw_zenoh_common_client.cpp
  src/rmw_zenoh_common_subscriber.cpp
  src/rmw_zenoh_common_wait_sets.cpp

  src/impl/wait_impl.cpp
  src/impl/pubsub_impl.cpp
  src/impl/service_impl.cpp
  src/impl/client_impl.cpp
  src/impl/type_support_common.cpp
  src/impl/qos.cpp
  src/impl/debug_helpers.cpp
)

ament_target_dependencies(rmw_zenoh_common_cpp
  rcutils
  rmw
  rosidl_typesupport_zenoh_c
  rosidl_typesupport_zenoh_cpp
  rosidl_generator_c
)
target_link_libraries(rmw_zenoh_common_cpp fastcdr)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rmw_zenoh_common_cpp PRIVATE "RMW_ZENOH_CPP_BUILDING_LIBRARY")
ament_export_include_directories(include)

ament_export_dependencies(rosidl_typesupport_zenoh_cpp)
ament_export_dependencies(rosidl_typesupport_zenoh_c)
ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rcutils)
ament_export_dependencies(rmw)

ament_export_libraries(rmw_zenoh_common_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS rmw_zenoh_common_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package(CONFIG_EXTRAS "rmw_zenoh_common_cpp-extras.cmake")
