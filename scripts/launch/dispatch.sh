#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# 场景3: 调度集成 (Dispatch Integration with openTCS)
#
# 启动硬件层 + EKF + AMCL定位 + Nav2导航 + openTCS车辆桥接
# 数据流: 传感器 → EKF → AMCL → Nav2 → /cmd_vel
#         opentcs_vehicle_node 上报位姿/电量 → openTCS Kernel
#         openTCS 下发路径 → opentcs_vehicle_node → Nav2 NavigateToPose
#
# 用法:
#   ./scripts/launch/dispatch.sh                                 # 默认: gazebo
#   ./scripts/launch/dispatch.sh --profile rs485                 # RS-485 实车
#   ./scripts/launch/dispatch.sh --profile raspberry             # 树莓派
#   ./scripts/launch/dispatch.sh --map /path/to/map.yaml         # 指定地图
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 默认参数
PROFILE="gazebo"
MAP_FILE="${PROJECT_DIR}/maps/auto_exploration_map.yaml"

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --map)
            MAP_FILE="$2"
            shift 2
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--profile gazebo|rs485|raspberry] [--map FILE]"
            exit 1
            ;;
    esac
done

# ── 环境初始化 ────────────────────────────────────────────────────
if [ "$PROFILE" = "raspberry" ]; then
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset ROS_DISCOVERY_SERVER
    source /opt/ros/jazzy/setup.bash
    source /home/pi/ros2_ws/install/setup.bash 2>/dev/null || true
    source "${PROJECT_DIR}/install/setup.bash"
else
    export DISPLAY=:0
    for auth in /run/user/$(id -u)/.mutter-Xwaylandauth.* /home/$(whoami)/.Xauthority; do
        if [ -f "$auth" ]; then
            export XAUTHORITY="$auth"
            break
        fi
    done

    eval "$(conda shell.bash hook)"
    conda activate lidar_slam
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset ROS_LOCALHOST_ONLY
    unset ROS_DISCOVERY_SERVER
    source "${PROJECT_DIR}/install/setup.bash"
fi

# ── 地图检查 ──────────────────────────────────────────────────────
if [ ! -f "$MAP_FILE" ]; then
    echo "ERROR: 地图文件不存在: $MAP_FILE"
    echo "用法: $0 [--map FILE]"
    exit 1
fi

# ── 日志 ──────────────────────────────────────────────────────────
LOG_DIR="${PROJECT_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/dispatch_${PROFILE}_$(date +%Y-%m-%d_%H-%M-%S).log"

echo "============================================="
echo "  调度集成场景"
echo "  Profile: $PROFILE"
echo "  地图:    $MAP_FILE"
echo "  日志:    ${LOG_FILE}"
echo "============================================="

# ── 清理残留 ──────────────────────────────────────────────────────
bash "${PROJECT_DIR}/scripts/tools/cleanup_ros2.sh"

# ── 启动 ──────────────────────────────────────────────────────────
exec ros2 launch "${PROJECT_DIR}/launch/nav_main.launch.py" \
    hardware_profile:="${PROFILE}" \
    use_respawn:=True \
    vehicle_name:=ackermann_robot \
    map_file:="$MAP_FILE" \
    < /dev/null \
    >> "${LOG_FILE}" 2>&1
