cmake_minimum_required(VERSION 3.5)
project(rmw_zenoh_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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

find_package(zenoh_pico_vendor REQUIRED)
find_package(zenoh_pico REQUIRED)
find_package(zenoh_vendor REQUIRED)
find_package(zenoh REQUIRED)

add_library(rmw_zenoh_common_cpp OBJECT
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
  src/rmw_publish.cpp
  src/rmw_publisher.cpp
  src/rmw_serialization.cpp
  src/rmw_service.cpp
  src/rmw_client.cpp
  src/rmw_subscriber.cpp
  src/rmw_wait_sets.cpp

  src/impl/wait_impl.cpp
  src/impl/pubsub_impl.cpp
  src/impl/service_impl.cpp
  src/impl/client_impl.cpp
  src/impl/type_support_common.cpp
  src/impl/qos.cpp
  src/impl/debug_helpers.cpp
)
set_property(TARGET rmw_zenoh_common_cpp PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(rmw_zenoh_pico_cpp
  src/rmw_zenoh_pico.c
  src/impl/identifier_zenoh_pico.c
  $<TARGET_OBJECTS:rmw_zenoh_common_cpp>
)
target_link_libraries(rmw_zenoh_pico_cpp
  fastcdr
  ${zenoh_pico_LIBRARIES}
)

add_library(rmw_zenoh_cpp
  src/rmw_zenoh_c.c
  src/impl/identifier_zenoh_c.c
  $<TARGET_OBJECTS:rmw_zenoh_common_cpp>
)
target_link_libraries(rmw_zenoh_cpp
  fastcdr
)

set(ZENOH_COMMON_TARGET_DEPENDENCIES
  rcutils
  rmw
  rosidl_typesupport_zenoh_c
  rosidl_typesupport_zenoh_cpp
  rosidl_generator_c
)

set(ZENOH_PICO_TARGET_DEPENDENCIES
  zenoh_pico
  zenoh_pico_vendor
  ${ZENOH_COMMON_TARGET_DEPENDENCIES}
)

set(ZENOH_C_TARGET_DEPENDENCIES
  zenoh
  zenoh_vendor
  ${ZENOH_COMMON_TARGET_DEPENDENCIES}
)

ament_target_dependencies(rmw_zenoh_common_cpp ${ZENOH_COMMON_TARGET_DEPENDENCIES})
ament_target_dependencies(rmw_zenoh_pico_cpp ${ZENOH_PICO_TARGET_DEPENDENCIES})
ament_target_dependencies(rmw_zenoh_cpp ${ZENOH_C_TARGET_DEPENDENCIES})

configure_rmw_library(rmw_zenoh_pico_cpp)
configure_rmw_library(rmw_zenoh_cpp)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rmw_zenoh_pico_cpp PRIVATE "RMW_ZENOH_CPP_BUILDING_LIBRARY")
target_compile_definitions(rmw_zenoh_cpp PRIVATE "RMW_ZENOH_CPP_BUILDING_LIBRARY")

ament_export_include_directories(include)
ament_export_libraries(rmw_zenoh_pico_cpp)
ament_export_libraries(rmw_zenoh_cpp)

ament_export_dependencies(rosidl_typesupport_zenoh_cpp)
ament_export_dependencies(rosidl_typesupport_zenoh_c)
ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rcutils)
ament_export_dependencies(rmw)
ament_export_dependencies(zenoh_pico_vendor)
ament_export_dependencies(zenoh_vendor)

register_rmw_implementation(
  "c:rosidl_typesupport_c:rosidl_typesupport_zenoh_c"
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_zenoh_cpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(osrf_testing_tools_cpp REQUIRED)

  ament_add_gtest(test_init_shutdown test/test_init_shutdown.cpp)
  ament_target_dependencies(test_init_shutdown rcutils)
  target_link_libraries(test_init_shutdown rmw_zenoh_cpp)

  ament_add_gtest(test_init_options test/test_init_options.cpp)
  ament_target_dependencies(test_init_options osrf_testing_tools_cpp rcutils)
  target_link_libraries(test_init_options rmw_zenoh_cpp)

  ament_add_gtest(test_create_destroy_node test/test_create_destroy_node.cpp)
  ament_target_dependencies(test_create_destroy_node rcutils)
  target_link_libraries(test_create_destroy_node rmw_zenoh_cpp)

  # Not yet implemented
  #ament_add_gtest(test_serialize_deserialize test/test_serialize_deserialize.cpp)
  #ament_target_dependencies(test_serialize_deserialize
    #rcutils
    #test_msgs
    #)
  #target_link_libraries(test_serialize_deserialize rmw_zenoh_cpp)

  # Broken by the type support build order problem
  #ament_add_gtest(test_publisher test/test_publisher.cpp)
  #ament_target_dependencies(test_publisher osrf_testing_tools_cpp rcutils test_msgs)
  #target_link_libraries(test_publisher rmw_zenoh_cpp)

  # Broken by the type support build order problem
  #ament_add_gtest(test_subscription test/test_subscription.cpp)
  #ament_target_dependencies(test_subscription osrf_testing_tools_cpp rcutils test_msgs)
  #target_link_libraries(test_subscription rmw_zenoh_cpp)
endif()

if(WITH_ZENOH_PICO)
  ament_package(
    CONFIG_EXTRAS_POST "rmw_zenoh_pico_cpp-extras.cmake"
  )
else()
  ament_package(
    CONFIG_EXTRAS_POST "rmw_zenoh_cpp-extras.cmake"
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rmw_zenoh_cpp rmw_zenoh_pico_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
