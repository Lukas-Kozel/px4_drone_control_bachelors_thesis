cmake_minimum_required(VERSION 3.5)
project(drone_pose_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/gz-cmake3/cmake3")
include(GzCMake)  # Example, include the main GzCMake module
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories("/usr/include/gz/sim7/gz/sim")
include_directories(${CMAKE_CURRENT_BINARY_DIR})  # This includes the build directory

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find Gazebo CMake helper and Gazebo simulator
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

set(
  plugins,
  drone_pose_plugin
)

# Add the protobuf generated file to the list of sources
# Assuming the generated files are in the same directory as the custom message
set(SOURCES
  drone_pose_plugin.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/drone_pose_stamped.pb.cc
)

add_library(drone_pose_plugin SHARED ${SOURCES})

ament_target_dependencies(drone_pose_plugin rclcpp geometry_msgs)

# Link against the Protocol Buffers library
find_package(Protobuf REQUIRED)
target_link_libraries(drone_pose_plugin 
  ${GAZEBO_LIBRARIES}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  ${PROTOBUF_LIBRARIES}
)

install(TARGETS drone_pose_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
