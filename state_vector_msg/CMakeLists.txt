cmake_minimum_required(VERSION 3.8)
project(state_vector_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(load_pose_stamped REQUIRED)
find_package(drone_pose_stamped REQUIRED)
find_package(angle_stamped_msg REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StateVector.msg"
  DEPENDENCIES angle_stamped_msg sensor_msgs drone_pose_stamped load_pose_stamped geometry_msgs
)



ament_package()
