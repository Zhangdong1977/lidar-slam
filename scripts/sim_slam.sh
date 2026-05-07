#!/bin/bash
# 启动 Gazebo 仿真 SLAM 管线
# 用法: ./scripts/sim_slam.sh

set -e

# GUI 程序需要
export DISPLAY=:0
for auth in /run/user/$(id -u)/.mutter-Xwaylandauth.* /home/$(whoami)/.Xauthority; do
    if [ -f "$auth" ]; then
        export XAUTHORITY="$auth"
        break
    fi
done

# ROS2 环境
source /opt/ros/jazzy/setup.bash

exec ros2 launch /home/pi/lidar-slam/launch/sim_slam.launch.py
