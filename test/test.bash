#!/bin/bash
# SPDX-FileCopyrightText: 2024 Youichi Masuyama <yaiti0212@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause



dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source $dir/.bashrc

# ノードを起動してログを確認
timeout 60 ros2 run mypkg resource_monitor > /tmp/mypkg.log

# トピック名や出力内容を正確に確認
cat /tmp/mypkg.log |
grep 'Published:'
