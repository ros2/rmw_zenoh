# Copyright 2020 ADLINK, Inc.
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

# copied from rmw_zenoh_dynamic_cpp/rmw_zenoh_dynamic_cpp-extras.cmake

find_package(zenoh_pico_vendor REQUIRED)
find_package(zenoh_pico REQUIRED)

list(APPEND rmw_zenoh_dynamic_cpp_INCLUDE_DIRS ${zenoh_pico_INCLUDE_DIRS})
# specific order: dependents before dependencies
list(APPEND rmw_zenoh_dynamic_cpp_LIBRARIES ${zenoh_pico_LIBRARIES})
