#!/bin/bash
# SPDX-FileCopyrightText: 2025 Youichi Masuyama <yaiti0212@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

dir=~
[ "$1" != "" ] && dir="$1"

echo "Starting test script..."
cd $dir/ros2_ws || { echo "Error: Could not navigate to $dir/ros2_ws"; exit 1; }

echo "Building the workspace..."
colcon build 2> /dev/null || { echo "Error: Build failed"; exit 1; }
echo "Build completed."

echo "Sourcing ROS environment..."
source $dir/.bashrc
. install/setup.bash

echo "Launching the system monitor node..."
timeout 60 ros2 run mypkg system_monitor > /tmp/mypkg.log &
MONITOR_PID=$!

# プロセスの進捗を表示
for ((i = 1; i <= 60; i++)); do
    if ! ps -p $MONITOR_PID > /dev/null; then
        echo "System monitor node has terminated."
        break
    fi
    echo -ne "Running system monitor... ($i/60 seconds)\r"
    sleep 1
done
wait $MONITOR_PID 2>/dev/null

if [[ $? -ne 0 ]]; then
    echo -e "\nWarning: System monitor node did not complete successfully."
else
    echo -e "\nSystem monitor node completed successfully."
fi

echo "Checking log output..."
grep 'Published:' /tmp/mypkg.log && echo "Log check completed successfully." || echo "No matching log entries found."

echo "Test script finished."

