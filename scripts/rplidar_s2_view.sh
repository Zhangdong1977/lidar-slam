#!/bin/bash
# 启动 RPLIDAR S2 驱动 (含 rviz2 可视化)
# 用法: ./scripts/rplidar_s2_view.sh [--no-gui]

set -e

# 自动检测串口设备
SERIAL_PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
if [ -z "$SERIAL_PORT" ]; then
    echo "[ERROR] 未找到串口设备 /dev/ttyUSB*"
    exit 1
fi
echo "[INFO] 使用串口: $SERIAL_PORT"

# 串口权限检查
if [ ! -w "$SERIAL_PORT" ]; then
    echo "[WARN] $SERIAL_PORT 无写入权限，尝试修复..."
    sudo chmod 666 "$SERIAL_PORT"
fi

# GUI 程序需要
export DISPLAY=:0
for auth in /run/user/$(id -u)/.mutter-Xwaylandauth.* /home/$(whoami)/.Xauthority; do
    if [ -f "$auth" ]; then
        export XAUTHORITY="$auth"
        break
    fi
done

# ROS2 环境
source /opt/ros/jazzy/setup.bash
source /home/pi/Desktop/code/lidar-slam/third-party/rplidar_ws/install/setup.bash

if [ "$1" = "--no-gui" ]; then
    exec ros2 launch rplidar_ros rplidar_s2.launch.py serial_port:=$SERIAL_PORT
else
    exec ros2 launch rplidar_ros view_rplidar_s2_launch.py serial_port:=$SERIAL_PORT
fi
