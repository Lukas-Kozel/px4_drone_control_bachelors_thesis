cmake_minimum_required(VERSION 3.8)
project(move_forward)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(px4_msgs REQUIRED)  # Add this line for px4_msgs dependency

# Add executable
add_executable(move_forward_node src/move_forward_node.cpp)
ament_target_dependencies(move_forward_node rclcpp px4_msgs geometry_msgs mavros_msgs)  # Add px4_msgs here

install(TARGETS
  move_forward_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
