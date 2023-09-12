cmake_minimum_required(VERSION 3.16)
project(zenoh_c_vendor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(zenoh_c_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
  VCS_VERSION master
)

# set(INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-prefix/install")
# install(
#   DIRECTORY "${INSTALL_DIR}/lib/"
#   DESTINATION "lib/${PROJECT_NAME}"
#   USE_SOURCE_PERMISSIONS
# )

ament_package()
