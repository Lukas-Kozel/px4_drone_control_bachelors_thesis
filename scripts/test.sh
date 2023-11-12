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
    kill_px4_processes
    kill_gazebo_processes
}

find_px4_pids(){
    px4_pids=$(ps aux | grep px4 | grep -v grep | awk '{print $2}')
    echo "$px4_pids" >> "$SIMULATION_PID_PATH"
}

kill_px4_processes(){
    px4_pids=$(ps aux | grep px4 | grep -v grep | awk '{print $2}')
    for pid in $px4_pids; do
        kill -SIGTERM "$pid" || echo "Failed to kill parameter_bridge with PID $pid"
    done
}

kill_gazebo_processes() {
    gazebo_pids=$(ps aux | grep "gz sim" | grep -v grep | awk '{print $2}')
    for pid in $gazebo_pids; do
        kill -SIGTERM "$pid" || echo "Failed to kill Gazebo process with PID $pid"
    done
}

# Array to store PIDs
declare -a pids
PID_LIST_PATH="/home/luky/mavros_ros2_ws/src/scripts/pid_list.txt"
SIMULATION_PID_PATH="/home/luky/mavros_ros2_ws/src/scripts/simulationPID.txt"
#empty files
> "$PID_LIST_PATH"
> "$SIMULATION_PID_PATH"
# Trap SIGINT and SIGTERM signals to run kill_processes function
trap kill_processes SIGINT SIGTERM

# Start processes and store their PIDs
(cd /home/luky/px4-SITL/PX4-Autopilot && make px4_sitl gz_x500) &
px4_pid=$!
    echo "$px4_pid" >> "$SIMULATION_PID_PATH"
pids+=($px4_pid)

(cd /home/luky/mavros_ros2_ws; ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14580) &
pids+=($!)
(cd /home/luky; ./QGroundControl.AppImage) &
pids+=($!)

sleep 2

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

find_px4_pids
for pid in "${pids[@]}"; do
    echo "$pid" >> "$PID_LIST_PATH"
done


# Wait for user to press Ctrl+C
wait
