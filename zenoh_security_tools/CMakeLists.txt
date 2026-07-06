cmake_minimum_required(VERSION 3.20)
project(zenoh_security_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# ament_cmake_ros_core provides the ament_ros_defaults target which sets the
# default C++ (C++20) and C (C17) standards for ROS packages.
find_package(ament_cmake_ros_core REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw_security_common REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(zenoh_cpp_vendor REQUIRED)

add_executable(generate_configs
    src/main.cpp
    src/config_generator.cpp
)
target_link_libraries(generate_configs PRIVATE
  nlohmann_json::nlohmann_json
  rcpputils::rcpputils
  rcutils::rcutils
  rmw_security_common::rmw_security_common_library
  tinyxml2::tinyxml2
  zenohcxx::zenohc
  ament_cmake_ros_core::ament_ros_defaults
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  TARGETS generate_configs
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
