cmake_minimum_required(VERSION 3.8)
project(drone_gui_node  VERSION 0.1 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

   
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(load_pose_stamped REQUIRED)
find_package(drone_pose_stamped REQUIRED)
find_package(angle_stamped_msg REQUIRED)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED) 
find_package(Qt5Charts REQUIRED)

# Qt related settings
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(gui_node src/gui_node.cpp)
add_executable(gui src/qt/main.cpp src/qt/MyWindow.cpp src/qt/ConnectionManager.cpp)

ament_target_dependencies(gui_node
  rclcpp 
  px4_msgs 
  mavros_msgs
  geometry_msgs
  load_pose_stamped 
  drone_pose_stamped 
  angle_stamped_msg
)
ament_target_dependencies(gui
  rclcpp 
  px4_msgs 
  mavros_msgs
  geometry_msgs
  load_pose_stamped 
  drone_pose_stamped 
  angle_stamped_msg
)


target_include_directories(gui PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${px4_msgs_INCLUDE_DIRS}
    ${mavros_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
)



target_link_libraries(gui
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Gui
  Qt${QT_VERSION_MAJOR}::Charts
  Qt${QT_VERSION_MAJOR}::Widgets
)

install(TARGETS
  gui_node
  gui
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize
if(QT_VERSION_MAJOR EQUAL 6)
  qt_finalize_executable(gui)
endif()
ament_package()
