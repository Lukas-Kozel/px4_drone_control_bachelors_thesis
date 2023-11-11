#!/bin/bash

# Function to kill processes
kill_processes() {
    echo "Terminating all processes..."
    for pid in "${pids[@]}"; do
        kill -SIGINT "$pid"
    done
}

# Array to store PIDs
declare -a pids

# Trap SIGINT and SIGTERM signals to run kill_processes function
trap kill_processes SIGINT SIGTERM

# Start processes and store their PIDs
gnome-terminal -- bash -c "cd /home/luky/px4-SITL/PX4-Autopilot; make px4_sitl gz_x500; exec bash" &
pids+=($!)
gnome-terminal -- bash -c "cd /home/luky/mavros_ros2_ws; ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14580; exec bash" &
pids+=($!)
gnome-terminal -- bash -c "cd /home/luky; ./QGroundControl.AppImage; exec bash" &
pids+=($!)

sleep 5

gnome-terminal -- bash -c "ros2 run ros_gz_bridge parameter_bridge /load_imu@sensor_msgs/msg/Imu@gz.msgs.IMU; exec bash" &
pids+=($!)
gnome-terminal -- bash -c "ros2 run ros_gz_bridge parameter_bridge /drone_imu@sensor_msgs/msg/Imu@gz.msgs.IMU; exec bash" &
pids+=($!)
gnome-terminal -- bash -c "ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock; exec bash" &
pids+=($!)

sleep 5

gnome-terminal -- bash -c "ros2 run gazebo_ros2_bridge bridge_node; exec bash" &
pids+=($!)
gnome-terminal -- bash -c "ros2 run angle_calculator angle_calculator_node; exec bash" &
pids+=($!)
gnome-terminal -- bash -c "ros2 run move_forward move_forward_node; exec bash" &
pids+=($!)

# Wait for user to press Ctrl+C
wait

