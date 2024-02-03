#!/bin/bash

# Source file path
SOURCE="/home/luky/mavros_ros2_ws/gz-plugins/gz_pose_plugin/libgz_pose_plugin.so"
# Target directory
TARGET_DIR="/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins"
# Target file path (symlink destination)
TARGET="$TARGET_DIR/libgz_pose_plugin.so"

# Check if the source file exists
if [ -f "$SOURCE" ]; then
    echo "Source file found."
    # Check if the target directory exists
    if [ -d "$TARGET_DIR" ]; then
        echo "Target directory exists. Creating symlink."
        # Create symlink with superuser permissions
        sudo ln -sfn $SOURCE $TARGET
        echo "Symlink created successfully."
    else
        echo "Target directory does not exist. Please check the path."
    fi
else
    echo "Source file does not exist. Please check the path."
fi
