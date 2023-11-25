
kill_gazebo_processes() {
    gazebo_pids=$(ps aux | grep "gz sim" | grep -v grep | awk '{print $2}')
    for pid in $gazebo_pids; do
        kill -SIGTERM "$pid" || echo "Failed to kill Gazebo process with PID $pid"
    done
}
kill_px4_processes(){
    px4_pids=$(ps aux | grep px4 | grep -v grep | awk '{print $2}')
    for pid in $px4_pids; do
        kill -SIGTERM "$pid" || echo "Failed to kill parameter_bridge with PID $pid"
    done
}
> "/home/luky/mavros_ros2_ws/src/scripts/px4_log.txt"
#kill all running processes
kill_px4_processes
kill_gazebo_processes

sleep 5
#run new instances, the output into file needs to e ethere or it do not work
(cd /home/luky/PX4-Autopilot  && make px4_sitl gz_x500)&> "/home/luky/mavros_ros2_ws/src/scripts/px4_log.txt" &

(cd /home/luky/mavros_ros2_ws; ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14580) &