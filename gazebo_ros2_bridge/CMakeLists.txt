cmake_minimum_required(VERSION 3.5)
project(gazebo_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  "/usr/include/gz/sim7/gz/sim"
  "/usr/include/gz/transport"
  "/home/luky/mavros_ros2_ws/gz-plugins/gz_pose_plugin"
  ${CMAKE_CURRENT_BINARY_DIR}
)

# Find packages
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(load_pose_stamped REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(Protobuf REQUIRED)

set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})
gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

# Source files
set(SOURCES
  src/bridge_node.cpp
  "/home/luky/mavros_ros2_ws/gz-plugins/gz_pose_plugin/load_pose_stamped.pb.cc"
)

# Add executable
add_executable(bridge_node ${SOURCES})

# Target include directories
target_include_directories(bridge_node
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${load_pose_stamped_INCLUDE_DIRS}
)

# Target link libraries
target_link_libraries(bridge_node 
  ${rclcpp_LIBRARIES} 
  ${load_pose_stamped_LIBRARIES} 
  ${GAZEBO_LIBRARIES} 
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER} 
  ${PROTOBUF_LIBRARIES}
)

# Install targets
install(TARGETS
  bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
