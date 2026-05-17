#!/bin/bash
# Ackermann 键盘遥控 (需在独立终端运行)
# 用法: ./scripts/launch/sim_ackermann_teleop.sh

eval "$(conda shell.bash hook)"
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash
source "${LIDAR_SLAM_ROOT:-/home/hello/lidar-slam}/install/setup.bash"

exec ros2 run lidar_slam_nodes ackermann_keyboard_teleop
