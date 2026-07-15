cmake_minimum_required(VERSION 3.20)
project(test_rmw_zenoh_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ament_cmake_ros_core provides the ament_ros_defaults target which sets the
  # default C++ (C++20) and C (C17) standards for ROS packages.
  find_package(ament_cmake_ros_core REQUIRED)
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
    ament_cmake_ros_core::ament_ros_defaults
  )
endif()

ament_package()
