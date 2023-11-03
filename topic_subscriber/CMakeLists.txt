cmake_minimum_required(VERSION 3.8)
project(topic_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(load_pose_stamped REQUIRED)

add_executable(listener src/topic_subscriber_node.cpp)

ament_target_dependencies(listener rclcpp load_pose_stamped)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
