#!/bin/bash

# Function to kill processes
kill_processes() {
    echo "Terminating all processes..."
    for pid in "${pids[@]}"; do
        kill -SIGINT "$pid"
    done
    parameter_bridge_pids=$(ps aux | grep parameter_bridge | grep -v grep | awk '{print $2}')

    for pid in $parameter_bridge_pids; do
        echo "Terminating parameter_bridge process with PID: $pid"
        kill -SIGTERM "$pid" || echo "Failed to kill parameter_bridge with PID $pid"
    done
}

# Array to store PIDs
declare -a pids
> pid_list.txt
# Trap SIGINT and SIGTERM signals to run kill_processes function
trap kill_processes SIGINT SIGTERM

# Start processes and store their PIDs
(cd /home/luky/px4-SITL/PX4-Autopilot; make px4_sitl gz_x500) &
pids+=($!)
(cd /home/luky/mavros_ros2_ws; ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14580) &
pids+=($!)
(cd /home/luky; ./QGroundControl.AppImage) &
pids+=($!)

sleep 5

ros2 run gazebo_ros2_bridge bridge_node &
pids+=($!)

ros2 run ros_gz_bridge parameter_bridge /load_imu@sensor_msgs/msg/Imu@gz.msgs.IMU &
pids+=($!)
ros2 run ros_gz_bridge parameter_bridge /drone_imu@sensor_msgs/msg/Imu@gz.msgs.IMU &
pids+=($!)
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock &
pids+=($!)

sleep 5

ros2 run gazebo_ros2_bridge bridge_node &
pids+=($!)
ros2 run angle_calculator angle_calculator_node &
pids+=($!)
ros2 run move_forward move_forward_node &
pids+=($!)

sleep 2 

for pid in "${pids[@]}"; do
    echo "$pid" >> pid_list.txt
done


# Wait for user to press Ctrl+C
wait
