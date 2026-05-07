#!/bin/bash
# 键盘遥控 (需在独立终端运行)
# 用法: ./scripts/sim_teleop.sh

set -e

source /opt/ros/jazzy/setup.bash

exec python3 /home/pi/lidar-slam/scripts/sim_teleop.py
