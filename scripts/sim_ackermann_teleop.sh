#!/bin/bash
# Ackermann 键盘遥控 (需在独立终端运行)
# 用法: ./scripts/sim_ackermann_teleop.sh

source /opt/ros/jazzy/setup.bash

python3 /home/pi/lidar-slam/scripts/ackermann_keyboard_teleop.py
