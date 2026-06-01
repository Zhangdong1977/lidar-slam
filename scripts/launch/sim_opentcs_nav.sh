#!/bin/bash
# sim 仿真 + openTCS 导航脚本
# 启动 sim 仿真车 + AMCL 定位 + Nav2 + openTCS 车辆桥接
#
# 场景定位: 仿真环境下对接 openTCS 调度系统
# 数据流: Gazebo → /scan + /odom + /imu → EKF → AMCL → Nav2
#         opentcs_vehicle_node 上报位姿/电量 → openTCS Kernel
#         openTCS 下发路径 → opentcs_vehicle_node → Nav2 NavigateToPose
# 适合: openTCS 调度算法仿真、车队调度验证
#
# 用法:
#   ./scripts/launch/sim_opentcs_nav.sh                                  # 默认地图
#   ./scripts/launch/sim_opentcs_nav.sh /home/hello/lidar-slam/maps/xxx.yaml  # 指定地图

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

# 地图文件 (默认或显式指定)
MAP_FILE="${1:-${LIDAR_SLAM_ROOT}/maps/auto_exploration_map.yaml}"
if [ ! -f "$MAP_FILE" ]; then
    echo "ERROR: 地图文件不存在: $MAP_FILE"
    echo "用法: $0 [map_file.yaml]"
    exit 1
fi

# 日志输出到 log 目录
LOG_DIR="${LIDAR_SLAM_ROOT}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/sim_opentcs_nav_$(date +%Y-%m-%d_%H-%M-%S).log"

echo "============================================="
echo "  sim openTCS 导航场景"
echo "  Profile: sim (仿真)"
echo "  Launch:  nav_main.launch.py"
echo "  地图:    $MAP_FILE"
echo "  日志:    ${LOG_FILE}"
echo "============================================="

# 清理残留进程
bash "${LIDAR_SLAM_ROOT}/scripts/tools/cleanup_ros2.sh"

# 启动 sim + 统一导航入口 (支持 openTCS 集成)
exec ros2 launch "${LIDAR_SLAM_ROOT}/launch/nav_main.launch.py" \
    hardware_profile:=gazebo \
    use_respawn:=True \
    vehicle_name:=ackermann_robot \
    map_file:="$MAP_FILE" \
    < /dev/null \
    >> "${LOG_FILE}" 2>&1
