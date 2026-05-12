#!/bin/bash
# 保存仿真地图
# 用法: ./scripts/sim_save_map.sh [地图名]

set -e

# 项目根目录: 优先使用环境变量，否则从脚本位置自动探测
: "${LIDAR_SLAM_ROOT:=$(cd "$(dirname "$0")/.." && pwd)}"

MAP_NAME="${1:-sim_test_map}"
MAP_DIR="$LIDAR_SLAM_ROOT/maps"

source /opt/ros/jazzy/setup.bash

echo "Saving map to ${MAP_DIR}/${MAP_NAME} ..."
ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}"
echo "Done: ${MAP_DIR}/${MAP_NAME}.pgm + ${MAP_DIR}/${MAP_NAME}.yaml"
