cmake_minimum_required(VERSION 3.5)
project(rmw_zenoh_cpp)

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

find_package(rmw_zenoh_common_cpp REQUIRED)

find_package(zenoh_vendor REQUIRED)
find_package(zenoh REQUIRED)

ament_export_dependencies(rosidl_typesupport_zenoh_cpp)
ament_export_dependencies(rosidl_typesupport_zenoh_c)
ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rcutils)
ament_export_dependencies(rmw)
ament_export_dependencies(rmw_zenoh_common_cpp)

register_rmw_implementation(
  "c:rosidl_typesupport_c:rosidl_typesupport_zenoh_c"
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_zenoh_cpp"
)

add_library(rmw_zenoh_cpp
  src/identifier.cpp
  src/rmw_zenoh.cpp
)
target_include_directories(rmw_zenoh_cpp PRIVATE include)
target_link_libraries(rmw_zenoh_cpp
  fastcdr
)

ament_target_dependencies(rmw_zenoh_cpp zenoh_vendor zenoh rmw_zenoh_common_cpp)
configure_rmw_library(rmw_zenoh_cpp)
ament_export_libraries(rmw_zenoh_cpp)
ament_export_dependencies(zenoh_vendor)

install(
  TARGETS rmw_zenoh_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(osrf_testing_tools_cpp REQUIRED)

  ament_add_gtest(test_init_shutdown test/test_init_shutdown.cpp)
  ament_target_dependencies(test_init_shutdown rcutils rmw_zenoh_common_cpp)
  target_link_libraries(test_init_shutdown rmw_zenoh_cpp)

  ament_add_gtest(test_init_options test/test_init_options.cpp)
  ament_target_dependencies(test_init_options osrf_testing_tools_cpp rcutils rmw_zenoh_common_cpp)
  target_link_libraries(test_init_options rmw_zenoh_cpp)

  ament_add_gtest(test_create_destroy_node test/test_create_destroy_node.cpp)
  ament_target_dependencies(test_create_destroy_node rcutils rmw_zenoh_common_cpp)
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

ament_package(
  CONFIG_EXTRAS_POST "rmw_zenoh_cpp-extras.cmake"
)
