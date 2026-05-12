#!/bin/bash
# 键盘遥控 (需在独立终端运行)
# 用法: ./scripts/sim_teleop.sh

set -e

# 项目根目录: 优先使用环境变量，否则从脚本位置自动探测
: "${LIDAR_SLAM_ROOT:=$(cd "$(dirname "$0")/.." && pwd)}"

source /opt/ros/jazzy/setup.bash

exec python3 "$LIDAR_SLAM_ROOT/scripts/sim_teleop.py"
