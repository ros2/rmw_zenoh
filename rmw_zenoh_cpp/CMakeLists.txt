cmake_minimum_required(VERSION 3.8)
project(rmw_zenoh_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(fastcdr CONFIG REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_typesupport_fastrtps_c REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(rmw REQUIRED)
find_package(zenoh_c_vendor REQUIRED)
find_package(zenohc REQUIRED)

add_library(rmw_zenoh_cpp SHARED
  src/detail/identifier.cpp
  src/detail/graph_cache.cpp
  src/detail/guard_condition.cpp
  src/detail/liveliness_utils.cpp
  src/detail/message_type_support.cpp
  src/detail/rmw_data_types.cpp
  src/detail/service_type_support.cpp
  src/detail/type_support.cpp
  src/detail/type_support_common.cpp
  src/detail/zenoh_config.cpp
  src/detail/zenoh_router_check.cpp
  src/rmw_event.cpp
  src/rmw_get_network_flow_endpoints.cpp
  src/rmw_get_node_info_and_types.cpp
  src/rmw_get_service_names_and_types.cpp
  src/rmw_get_topic_endpoint_info.cpp
  src/rmw_get_topic_names_and_types.cpp
  src/rmw_init_options.cpp
  src/rmw_init.cpp
  src/rmw_qos.cpp
  src/rmw_zenoh.cpp
)

target_link_libraries(rmw_zenoh_cpp
  PRIVATE
    ament_index_cpp::ament_index_cpp
    fastcdr
    rcpputils::rcpputils
    rcutils::rcutils
    rosidl_typesupport_fastrtps_c::rosidl_typesupport_fastrtps_c
    rosidl_typesupport_fastrtps_cpp::rosidl_typesupport_fastrtps_cpp
    rmw::rmw
    zenohc::lib
)

configure_rmw_library(rmw_zenoh_cpp)

target_compile_definitions(rmw_zenoh_cpp
  PRIVATE
    RMW_VERSION_MAJOR=${rmw_VERSION_MAJOR}
    RMW_VERSION_MINOR=${rmw_VERSION_MINOR}
    RMW_VERSION_PATCH=${rmw_VERSION_PATCH}
)

ament_export_targets(export_rmw_zenoh_cpp)

register_rmw_implementation(
  "c:rosidl_typesupport_c:rosidl_typesupport_fastrtps_c:rosidl_typesupport_introspection_c"
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_fastrtps_cpp:rosidl_typesupport_introspection_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS rmw_zenoh_cpp
  EXPORT export_rmw_zenoh_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(init_rmw_zenoh_router apps/init_rmw_zenoh_router.cpp)

target_link_libraries(init_rmw_zenoh_router
  PRIVATE
    ament_index_cpp::ament_index_cpp
    zenohc::lib
)

install(
  TARGETS init_rmw_zenoh_router
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
