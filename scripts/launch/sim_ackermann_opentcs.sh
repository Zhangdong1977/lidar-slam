#!/bin/bash
# 启动 Ackermann 仿真导航 + openTCS 桥接
# 用法: ./scripts/launch/sim_ackermann_opentcs.sh

set -e

# GUI 程序需要
export DISPLAY=:0
for auth in /run/user/$(id -u)/.mutter-Xwaylandauth.* /home/$(whoami)/.Xauthority; do
    if [ -f "$auth" ]; then
        export XAUTHORITY="$auth"
        break
    fi
done

# ROS2 环境
eval "$(conda shell.bash hook)"
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER
LIDAR_SLAM_ROOT="${LIDAR_SLAM_ROOT:-/home/hello/lidar-slam}"
source "${LIDAR_SLAM_ROOT}/install/setup.bash"

# 清理残留进程，避免 Gazebo/ROS2 僵尸进程导致启动失败
bash "${LIDAR_SLAM_ROOT}/scripts/tools/cleanup_ros2.sh"

exec ros2 launch "${LIDAR_SLAM_ROOT}/launch/sim_ackermann_opentcs.launch.py"
