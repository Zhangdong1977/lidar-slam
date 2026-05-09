#!/bin/bash
# 保存仿真地图
# 用法: ./scripts/sim_save_map.sh [地图名]

set -e

MAP_NAME="${1:-sim_test_map}"
MAP_DIR="/home/pi/Desktop/code/lidar-slam/maps"

source /opt/ros/jazzy/setup.bash

echo "Saving map to ${MAP_DIR}/${MAP_NAME} ..."
ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}"
echo "Done: ${MAP_DIR}/${MAP_NAME}.pgm + ${MAP_DIR}/${MAP_NAME}.yaml"
