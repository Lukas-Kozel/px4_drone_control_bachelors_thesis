cmake_minimum_required(VERSION 3.8)
project(angle_calculator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angle_stamped_msg REQUIRED)
find_package(rosgraph_msgs REQUIRED)


add_executable(angle_calculator_node src/angle_calculator_node.cpp)
ament_target_dependencies(angle_calculator_node rclcpp std_msgs geometry_msgs tf2 tf2_geometry_msgs angle_stamped_msg rosgraph_msgs)



install(TARGETS
  angle_calculator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
