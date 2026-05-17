#!/bin/bash
# 保存 Ackermann 仿真地图
# 用法: ./scripts/sim_ackermann_save_map.sh [地图名]

set -e

MAP_NAME="${1:-ackermann_map}"
LIDAR_SLAM_ROOT="${LIDAR_SLAM_ROOT:-/home/hello/lidar-slam}"
MAP_DIR="${LIDAR_SLAM_ROOT}/maps"

eval "$(conda shell.bash hook)"
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash

echo "Saving map to ${MAP_DIR}/${MAP_NAME} ..."
ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}"
echo "Done: ${MAP_DIR}/${MAP_NAME}.pgm + ${MAP_DIR}/${MAP_NAME}.yaml"
