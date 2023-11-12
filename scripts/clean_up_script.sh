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

#kill all running processes
kill_px4_processes
kill_gazebo_processes


echo "cleaned up"