#!/bin/bash

#open px4-SITL
gnome-terminal -- bash -c "cd /home/luky/px4-SITL/PX4-Autopilot; make px4_sitl gz_x500; exec bash"

#open mavros
gnome-terminal -- bash -c "cd /home/luky/mavros_ros2_ws;ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14580; exec bash"

#open QgroundControl.AppImage
gnome-terminal -- bash -c "cd /home/luky; ./QGroundControl.AppImage; exec bash"

sleep 5

gnome-terminal -- bash -c "ros2 run ros_gz_bridge parameter_bridge /load_imu@sensor_msgs/msg/Imu@gz.msgs.IMU;exec bash"

gnome-terminal -- bash -c "ros2 run ros_gz_bridge parameter_bridge /drone_imu@sensor_msgs/msg/Imu@gz.msgs.IMU;exec bash"

gnome-terminal -- bash -c "ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock;exec bash"

sleep 5

gnome-terminal -- bash -c "ros2 run gazebo_ros2_bridge bridge_node;exec bash"

gnome-terminal -- bash -c "ros2 run angle_calculator angle_calculator_node;exec bash"

#optional, remove when controller is working correctly
gnome-terminal -- bash -c "ros2 run move_forward move_forward_node;exec bash"
