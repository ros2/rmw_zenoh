cmake_minimum_required(VERSION 3.20)
project(test_rmw_zenoh_cpp)

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rmw_zenoh_cpp REQUIRED)
  find_package(zenoh_cpp_vendor REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_rmw_zenoh_session
    test/test_rmw_zenoh_session.cpp
    ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp)
  target_link_libraries(test_rmw_zenoh_session
    rclcpp::rclcpp
    rmw_zenoh_cpp::rmw_zenoh_cpp
    zenohcxx::zenohc
  )
endif()

ament_package()
