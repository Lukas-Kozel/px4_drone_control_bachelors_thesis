cmake_minimum_required(VERSION 3.8)
project(lqr_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(
    include
    ${Eigen_INCLUDE_DIRS}
    #/home/luky/mavros_ros2_ws/install/mavros_msgs/include
)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(load_pose_stamped REQUIRED)
find_package(drone_pose_stamped REQUIRED)
find_package(angle_stamped_msg REQUIRED)
find_package(yaml-cpp REQUIRED)
add_executable(lqr_controller_node src/lqr_controller_node.cpp src/PIDController.cpp)
ament_target_dependencies(lqr_controller_node rclcpp std_msgs geometry_msgs sensor_msgs mavros_msgs tf2 tf2_geometry_msgs Eigen3 load_pose_stamped drone_pose_stamped  angle_stamped_msg)
target_link_libraries(lqr_controller_node ${Eigen_LIBRARIES} yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  lqr_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
