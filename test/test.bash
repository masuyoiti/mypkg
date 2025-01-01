#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws

# ROS 2 環境の初期化
source /opt/ros/humble/setup.bash || { echo "ERROR: Failed to source ROS 2 setup.bash."; exit 1; }
source install/setup.bash || { echo "ERROR: Failed to source workspace setup.bash."; exit 1; }

# ビルド
if ! colcon build; then
  echo "ERROR: colcon build failed."
  exit 1
fi

# ノードを起動してログを確認
if ! timeout 10 ros2 run mypkg resource_monitor > /tmp/mypkg.log; then
  echo "ERROR: Failed to run resource_monitor node."
  echo "Contents of /tmp/mypkg.log:"
  cat /tmp/mypkg.log  # ログ内容を出力
  exit 1
fi

# トピック名や出力内容を正確に確認
if ! grep 'Published:' /tmp/mypkg.log; then
  echo "ERROR: Expected output not found in /tmp/mypkg.log."
  echo "Contents of /tmp/mypkg.log:"
  cat /tmp/mypkg.log  # ログ内容を出力
  exit 1
fi

echo "Test completed successfully."
exit 0

