#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws

# ROS 2 環境の初期化
source /opt/ros/humble/setup.bash  # ROS 2 Humble を使用している場合
source install/setup.bash          # ROS 2 ワークスペースの環境を追加

colcon build

# ノードを起動してログを確認
timeout 10 ros2 run mypkg resource_monitor > /tmp/mypkg.log

# トピック名や出力内容を正確に確認
cat /tmp/mypkg.log |
grep 'Published:'

