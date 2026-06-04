#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# 键盘遥控 (Keyboard Teleop)
#
# 启动 ackermann_keyboard_teleop 节点, 用于手动遥控 AGV
# 需要在独立终端运行 (需要键盘输入)
#
# 用法:
#   ./scripts/launch/teleop.sh
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 默认参数
DOMAIN_ID=""
NAMESPACE=""

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --domain-id)
            DOMAIN_ID="$2"
            shift 2
            ;;
        --namespace)
            NAMESPACE="$2"
            shift 2
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--domain-id ID] [--namespace NS]"
            exit 1
            ;;
    esac
done

# ROS2 环境
eval "$(conda shell.bash hook)"
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${DOMAIN_ID:-${ROS_DOMAIN_ID:-42}}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
unset ROS_DISCOVERY_SERVER
source "${PROJECT_DIR}/install/setup.bash"

echo "=== 键盘遥控模式 ==="
echo "使用方向键或 WASD 控制车辆"
echo "Namespace: ${NAMESPACE:-无}"
echo "按 Ctrl+C 退出"
echo "===================="

if [ -n "$NAMESPACE" ]; then
    exec ros2 run lidar_slam_nodes ackermann_keyboard_teleop --ros-args -r __ns:=/$NAMESPACE
else
    exec ros2 run lidar_slam_nodes ackermann_keyboard_teleop
fi
