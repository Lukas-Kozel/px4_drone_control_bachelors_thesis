#!/bin/bash

echo "Starting script..."

# Function to start tmux session from YAML config
start_tmux_session() {
    local config_file="config.yaml"
    echo "Loading session name from $config_file"
    # Adjusted yq command syntax for version 3
    local session_name=$(yq '.session_name' $config_file)
    echo "Session name: $session_name"

    echo "Checking for existing tmux sessions..."
    if tmux has-session -t "$session_name" 2>/dev/null; then
        echo "Session $session_name already exists. Exiting..."
        exit 1
    fi

    echo "Starting new tmux session: $session_name"
    tmux new-session -d -s "$session_name"

    # Adjusted yq command syntax for version 3
    local windows=$(yq '.windows | length' $config_file)
    echo "Number of windows to create: $windows"
    for ((i = 0 ; i < $windows ; i++ )); do
        # Adjusted yq command syntax for version 3
        local window_name=$(yq ".windows[$i].window_name" $config_file)
        # Adjusted yq command syntax for version 3
        local layout=$(yq ".windows[$i].layout" $config_file)
        echo "Configuring window $i: $window_name with layout: $layout"

        [ $i -eq 0 ] && tmux rename-window "$window_name" || tmux new-window -n "$window_name"
        tmux select-layout "$layout"

        # Adjusted yq command syntax for version 3
        local panes=$(yq ".windows[$i].panes | length" $config_file)
        echo "Number of panes in window $window_name: $panes"
        for ((j = 0 ; j < $panes ; j++ )); do
            # Adjusted yq command syntax for version 3
            local directory=$(yq ".windows[$i].panes[$j].directory" $config_file)
            # Adjusted yq command syntax for version 3
            local command=$(yq ".windows[$i].panes[$j].command" $config_file)
            echo "Executing command in pane $j of window $window_name: $command"

            [ $j -ne 0 ] && tmux split-window
            tmux send-keys -t "$session_name:$i.$j" "cd $directory && $command & echo \$! >> $PID_LIST_PATH" C-m
        done

        tmux select-layout "$layout"
    done

    tmux attach -t "$session_name"
}

echo "Initiating tmux session..."

# Function to kill processes
kill_processes() {
    echo "Terminating all processes..."
    while read pid; do
        kill -SIGTERM "$pid" || echo "Failed to kill process with PID $pid"
    done < "$PID_LIST_PATH"
    # Additional kill logic as required
}

# Paths for PID storage
PID_LIST_PATH="/home/luky/mavros_ros2_ws/src/scripts/pid_list.txt"

# Empty the PID file
> "$PID_LIST_PATH"

# Trap SIGINT and SIGTERM signals to run kill_processes function
trap kill_processes SIGINT SIGTERM

# Start the tmux session with configurations
start_tmux_session

# Wait for Ctrl+C
wait
