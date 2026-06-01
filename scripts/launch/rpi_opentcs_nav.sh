#!/bin/bash
# 树莓派小车 openTCS 导航脚本
# 启动 RPLIDAR C1 + car_base_node (STM32) + AMCL 定位 + Nav2 + openTCS 车辆桥接
#
# 场景定位: 真实小车对接 openTCS 调度系统
# 数据流: 传感器 → EKF → AMCL → Nav2
#         car_base_node 订阅 /cmd_vel → STM32 UART → 电机 (Ackermann 解算在 STM32 内部)
#         opentcs_vehicle_node 上报位姿/电量 → openTCS Kernel
#         openTCS 下发路径 → opentcs_vehicle_node → Nav2 NavigateToPose
# 适合: 真实环境车队调度
#
# 硬件要求:
#   RPLIDAR C1: /dev/lidar (460800 baud)
#   STM32:      /dev/ttyAMA0 (115200 baud)
#
# 配套: 另开终端启动 openTCS Kernel + PlantOverview
#   cd third-party/openTCS-NeNa
#   ./gradlew :openTCS-NeNa-Kernel:run
#   ./gradlew :openTCS-NeNa-PlantOverview:run
#
# 用法:
#   ./scripts/launch/rpi_opentcs_nav.sh                          # 默认地图
#   ./scripts/launch/rpi_opentcs_nav.sh /path/to/raspberry_map.yaml

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# ROS2 环境
export LIDAR_SLAM_ROOT="${LIDAR_SLAM_ROOT:-$PROJECT_DIR}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER

source /opt/ros/jazzy/setup.bash
source /home/pi/ros2_ws/install/setup.bash 2>/dev/null || true
source "${PROJECT_DIR}/install/setup.bash"

# 串口设备检查
LIDAR_PORT="${LIDAR_PORT:-/dev/lidar}"
CHASSIS_PORT="${CHASSIS_PORT:-/dev/ttyAMA0}"
for dev in "$LIDAR_PORT" "$CHASSIS_PORT"; do
    if [ ! -e "$dev" ]; then
        echo "ERROR: 串口设备不存在: $dev"
        exit 1
    fi
    if [ ! -r "$dev" ] || [ ! -w "$dev" ]; then
        sudo chmod 666 "$dev" 2>/dev/null || \
            echo "WARN: 请手动执行 sudo chmod 666 $dev"
    fi
done

# 地图文件
MAP_FILE="${1:-${LIDAR_SLAM_ROOT}/maps/auto_exploration_map.yaml}"
if [ ! -f "$MAP_FILE" ]; then
    echo "ERROR: 地图文件不存在: $MAP_FILE"
    echo "用法: $0 [map_file.yaml]"
    exit 1
fi

# 日志输出到 log 目录
LOG_DIR="${LIDAR_SLAM_ROOT}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/rpi_opentcs_nav_$(date +%Y-%m-%d_%H-%M-%S).log"

echo "============================================="
echo "  树莓派小车 openTCS 导航场景"
echo "  Profile:  raspberry (实车)"
echo "  雷达:     $LIDAR_PORT (RPLIDAR C1)"
echo "  底盘:     $CHASSIS_PORT (STM32)"
echo "  地图:     $MAP_FILE"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  日志:     ${LOG_FILE}"
echo "============================================="

# 清理残留进程
bash "${SCRIPT_DIR}/../tools/cleanup_ros2.sh" 2>/dev/null || true

# 启动统一导航入口 (profile=raspberry, 跳过 cmd_vel_bridge / rs485_bridge)
exec ros2 launch "${LIDAR_SLAM_ROOT}/launch/nav_main.launch.py" \
    hardware_profile:=raspberry \
    use_respawn:=True \
    vehicle_name:=raspberry_agv \
    map_file:="$MAP_FILE" \
    < /dev/null \
    >> "${LOG_FILE}" 2>&1
