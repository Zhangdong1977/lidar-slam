#!/bin/bash
# 启动 Ackermann 仿真 SLAM 管线
# 用法: ./scripts/sim_ackermann_slam.sh

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
eval "$(conda shell.bash hook)"
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash
LIDAR_SLAM_ROOT="${LIDAR_SLAM_ROOT:-/home/hello/lidar-slam}"
source "${LIDAR_SLAM_ROOT}/install/setup.bash"

exec ros2 launch "${LIDAR_SLAM_ROOT}/launch/sim_ackermann.launch.py"
