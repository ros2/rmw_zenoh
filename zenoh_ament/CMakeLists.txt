cmake_minimum_required(VERSION 3.5)
project(zenoh_ament)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(include)

# Install lib
install(FILES
  ${CMAKE_SOURCE_DIR}/lib/libzenoh_ffi.so
  DESTINATION lib
)
ament_export_libraries(${CMAKE_SOURCE_DIR}/lib/libzenoh_ffi.so)

# Install /include Headers
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/zenoh/
  DESTINATION include/zenoh
)

ament_export_include_directories(include)
ament_package()
