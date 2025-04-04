cmake_minimum_required(VERSION 3.8)
project(zenoh_security_configuration_tools)

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
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(CLI11 REQUIRED)
find_package(zenoh_cpp_vendor REQUIRED)

add_executable(zenoh_security_configuration_tools
    src/zenoh_security_configuration_tools.cpp
    src/policy_parser.cpp
)
target_link_libraries(zenoh_security_configuration_tools PRIVATE
  CLI11::CLI11
  nlohmann_json::nlohmann_json
  tinyxml2::tinyxml2
  zenohcxx::zenohc
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  if(WIN32)
target_compile_definitions(${PROJECT_NAME}
    PRIVATE "ZENOH_SECURITY_CONFIGURATION_TOOLS_BUILDING_LIBRARY")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  TARGETS zenoh_security_configuration_tools
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
