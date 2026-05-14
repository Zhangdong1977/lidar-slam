#!/bin/bash
# Ackermann 键盘遥控 (需在独立终端运行)
# 用法: ./scripts/sim_ackermann_teleop.sh

eval "$(conda shell.bash hook)"
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash

LIDAR_SLAM_ROOT="${LIDAR_SLAM_ROOT:-/home/hello/lidar-slam}"
python3 "${LIDAR_SLAM_ROOT}/scripts/ackermann_keyboard_teleop.py"
