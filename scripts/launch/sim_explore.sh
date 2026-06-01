#!/bin/bash
# sim 仿真自动探索建图脚本
# 启动 sim 仿真车 + slam_toolbox + frontier_explorer 自动探索
#
# 场景定位: 自主探索建图 (无 Nav2 BT, 无 openTCS)
# 数据流: Gazebo → /scan + /odom + /imu → slam_toolbox → /map
#         frontier_explorer 自动选择 frontier → 2D Nav Goal → Nav2 → /cmd_vel → 底盘
# 适合: 未知环境自动建图
#
# 探索完成后保存地图:
#   ros2 run nav2_map_server map_saver_cli -f ~/maps/gazebo_auto_map
#
# 用法:
#   ./scripts/launch/sim_explore.sh

set -e

# GUI 程序需要 (X11/Wayland)
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
LOG_FILE="${LOG_DIR}/sim_explore_$(date +%Y-%m-%d_%H-%M-%S).log"

echo "============================================="
echo "  sim 自动探索建图场景"
echo "  Profile: sim (仿真)"
echo "  Launch:  sim_ackermann_explore.launch.py"
echo "  日志:    ${LOG_FILE}"
echo "============================================="

# 清理残留进程
bash "${LIDAR_SLAM_ROOT}/scripts/tools/cleanup_ros2.sh"

# 启动 sim + 探索
exec ros2 launch "${LIDAR_SLAM_ROOT}/launch/sim_ackermann_explore.launch.py" "$@" \
    < /dev/null \
    >> "${LOG_FILE}" 2>&1
