#!/bin/bash
# 真实小车导航启动脚本（无 Gazebo，无 socat）
# 用法: ./scripts/launch/real_ackermann_nav.sh
#
# 与 sim_ackermann_rs485.sh 共享同一个 launch 文件，
# 通过 simulation:=False 关闭仿真专用节点（Gazebo/socat/spawn_robot/load_controllers/rs485_receiver/vehicle_controller）

set -e

# ROS2 环境
eval "$(conda shell.bash hook)"
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER
LIDAR_SLAM_ROOT="${LIDAR_SLAM_ROOT:-/home/hello/lidar-slam}"
source "${LIDAR_SLAM_ROOT}/install/setup.bash"

# 日志输出到 log 目录
LOG_DIR="${LIDAR_SLAM_ROOT}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/real_nav_$(date +%Y-%m-%d_%H-%M-%S).log"

echo "日志文件: ${LOG_FILE}"

# 清理残留进程
bash "${LIDAR_SLAM_ROOT}/scripts/tools/cleanup_ros2.sh"

exec ros2 launch "${LIDAR_SLAM_ROOT}/launch/sim_ackermann_rs485.launch.py" \
    simulation:=False \
    use_sim_time:=False \
    "$@" \
    < /dev/null \
    >> "${LOG_FILE}" 2>&1
