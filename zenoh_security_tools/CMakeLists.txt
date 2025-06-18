cmake_minimum_required(VERSION 3.8)
project(zenoh_security_tools)

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
find_package(nlohmann_json REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
# todo(Yadunund): Remove rmw dependency after https://github.com/ros2/rmw/pull/400 is merged.
find_package(rmw REQUIRED)
find_package(rmw_dds_common REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
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
  rmw::rmw
  rmw_dds_common::rmw_dds_common_library
  tinyxml2::tinyxml2
  zenohcxx::zenohc
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
