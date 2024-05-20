find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(
  test_ring_buffer
  test_ring_buffer.cpp
)
