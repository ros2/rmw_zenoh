# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# copied from rmw_fastrtps_cpp/rmw_fastrtps_cpp-extras.cmake

find_package(zenoh_vendor QUIET)
if(zenoh_vendor_FOUND)
  find_package(zenoh QUIET)
  if(zenoh_FOUND)
    list(APPEND rmw_zenoh_cpp_INCLUDE_DIRS ${zenoh_INCLUDE_DIRS})
    # specific order: dependents before dependencies
    list(APPEND rmw_zenoh_cpp_LIBRARIES ${zenoh_LIBRARIES})
  endif()
endif()

find_package(zenoh_pico_vendor QUIET)
if(zenoh_pico_vendor_FOUND)
  find_package(zenoh_pico QUIET)
  if(zenoh_pico_FOUND)
    list(APPEND rmw_zenoh_cpp_INCLUDE_DIRS ${zenoh_pico_INCLUDE_DIRS})
    # specific order: dependents before dependencies
    list(APPEND rmw_zenoh_cpp_LIBRARIES ${zenoh_pico_LIBRARIES})
  endif()
endif()

if(NOT zenoh_FOUND AND NOT zenoh_pico_FOUND)
  message(FATAL_ERROR "rmw_zenoh_cpp needs at least zenoh-c or zenoh-pico, could not find either")
endif()
