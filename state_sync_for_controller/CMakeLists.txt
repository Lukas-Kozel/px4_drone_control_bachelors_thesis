cmake_minimum_required(VERSION 3.8)
project(state_sync_for_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(state_vector_msg REQUIRED)
find_package(message_filters REQUIRED)


add_executable(state_sync_for_controller_node src/state_sync_for_controller_node.cpp)
ament_target_dependencies(state_sync_for_controller_node rclcpp std_msgs geometry_msgs sensor_msgs message_filters state_vector_msg)

install(TARGETS
  state_sync_for_controller_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
