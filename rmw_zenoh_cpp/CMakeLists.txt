cmake_minimum_required(VERSION 3.5)
project(rmw_zenoh_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE _CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)

find_package(zenoh_ament REQUIRED)

include_directories(include)

add_library(rmw_zenoh_cpp
  src/identifier.cpp
  src/rmw_event.cpp
  src/rmw_get_node_info_and_types.cpp
  src/rmw_get_service_names_and_types.cpp
  src/rmw_get_topic_names_and_types.cpp
  src/rmw_get_topic_endpoint_info.cpp
  src/rmw_graph_info.cpp
  src/rmw_gids.cpp
  src/rmw_guard_conditions.cpp
  src/rmw_implementation_info.cpp
  src/rmw_init.cpp
  src/rmw_init_options.cpp
  src/rmw_logging.cpp
  src/rmw_node.cpp
  src/rmw_publisher.cpp
  src/rmw_serialization.cpp
  src/rmw_services.cpp
  src/rmw_subscriber.cpp
  src/rmw_wait_sets.cpp
)
ament_target_dependencies(rmw_zenoh_cpp
  rcutils
  rmw
  zenoh_ament
)

configure_rmw_library(rmw_zenoh_cpp)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RMW_ZENOH_CPP_BUILDING_LIBRARY")

ament_export_include_directories(include)
ament_export_libraries(rmw_zenoh_cpp)
ament_export_dependencies(rcutils)
ament_export_dependencies(rmw)
ament_export_dependencies(zenoh_ament)

register_rmw_implementation(
  "c:rosidl_typesupport_c"
  "cpp:rosidl_typesupport_cpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rmw_zenoh_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
