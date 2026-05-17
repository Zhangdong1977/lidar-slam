#!/bin/bash
# 键盘遥控 (需在独立终端运行)
# 用法: ./scripts/launch/sim_teleop.sh

set -e

: "${LIDAR_SLAM_ROOT:=$(cd "$(dirname "$0")/../.." && pwd)}"

eval "$(conda shell.bash hook)"
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash
source "${LIDAR_SLAM_ROOT}/install/setup.bash"

exec ros2 run lidar_slam_nodes sim_teleop
