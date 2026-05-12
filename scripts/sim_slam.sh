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

# 项目根目录: 优先使用环境变量，否则从脚本位置自动探测
: "${LIDAR_SLAM_ROOT:=$(cd "$(dirname "$0")/.." && pwd)}"
export LIDAR_SLAM_ROOT
echo "[INFO] LIDAR_SLAM_ROOT=$LIDAR_SLAM_ROOT"

# ROS2 环境
source /opt/ros/jazzy/setup.bash

LAUNCH_FILE="$LIDAR_SLAM_ROOT/launch/sim_slam.launch.py"

exec ros2 launch "$LAUNCH_FILE"
