#!/bin/bash
# 启动 Ackermann 仿真自动探索建图管线
# 用法: ./scripts/sim_ackermann_explore.sh

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
export ROS_LOCALHOST_ONLY=1
unset ROS_DISCOVERY_SERVER
LIDAR_SLAM_ROOT="${LIDAR_SLAM_ROOT:-/home/hello/lidar-slam}"
source "${LIDAR_SLAM_ROOT}/install/setup.bash"

# 日志输出到 log 目录
LOG_DIR="${LIDAR_SLAM_ROOT}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/explore_$(date +%Y-%m-%d_%H-%M-%S).log"

echo "日志文件: ${LOG_FILE}"

# 清理残留进程，避免 Gazebo/ROS2 僵尸进程导致启动失败
bash "${LIDAR_SLAM_ROOT}/scripts/tools/cleanup_ros2.sh"

exec ros2 launch "${LIDAR_SLAM_ROOT}/launch/sim_ackermann_explore.launch.py" "$@" \
    < /dev/null \
    >> "${LOG_FILE}" 2>&1
