#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# 场景2: 自动探索建图 (Autonomous Exploration Mapping)
#
# 启动硬件层 + EKF + slam_toolbox + Nav2 + frontier_explorer
# 数据流: 传感器 → EKF → slam_toolbox → /map + TF map→odom
#         frontier_explorer → NavigateToPose → Nav2 → /cmd_vel → 底盘
#
# 用法:
#   ./scripts/launch/explore.sh                  # 默认: gazebo 仿真
#   ./scripts/launch/explore.sh --profile raspberry
#   ./scripts/launch/explore.sh --nav2-params /path/to/params
#   ./scripts/launch/explore.sh --no-rviz              # 不启动 RViz
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 默认参数
PROFILE="gazebo"
NAV2_PARAMS=""
SLAM_PARAMS=""
DOMAIN_ID=""
USE_RVIZ="True"
NAMESPACE=""

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --nav2-params)
            NAV2_PARAMS="$2"
            shift 2
            ;;
        --slam-params)
            SLAM_PARAMS="$2"
            shift 2
            ;;
        --domain-id)
            DOMAIN_ID="$2"
            shift 2
            ;;
        --no-rviz)
            USE_RVIZ="False"
            shift
            ;;
        --namespace)
            NAMESPACE="$2"
            shift 2
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--profile gazebo|rs485|raspberry] [--nav2-params FILE] [--slam-params FILE] [--no-rviz] [--domain-id ID] [--namespace NS]"
            exit 1
            ;;
    esac
done

# ── 环境初始化 ────────────────────────────────────────────────────
# 从 profile YAML 读取 domain_id（默认 42）
DEFAULT_DOMAIN=$(python3 -c "import yaml; print(yaml.safe_load(open('${PROJECT_DIR}/config/profiles/${PROFILE}.yaml')).get('domain_id', 42))" 2>/dev/null || echo 42)
export ROS_DOMAIN_ID="${DOMAIN_ID:-$DEFAULT_DOMAIN}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [ "$PROFILE" = "raspberry" ]; then
    unset ROS_DISCOVERY_SERVER
    source /opt/ros/jazzy/setup.bash
    source /home/pi/ros2_ws/install/setup.bash 2>/dev/null || true
    source "${PROJECT_DIR}/install/setup.bash"

    LIDAR_PORT="${LIDAR_PORT:-/dev/lidar}"
    CHASSIS_PORT="${CHASSIS_PORT:-/dev/ttyAMA0}"
    for dev in "$LIDAR_PORT" "$CHASSIS_PORT"; do
        if [ ! -e "$dev" ]; then
            echo "ERROR: 串口设备不存在: $dev"
            exit 1
        fi
        if [ ! -r "$dev" ] || [ ! -w "$dev" ]; then
            sudo chmod 666 "$dev" 2>/dev/null || echo "WARN: 请手动 chmod $dev"
        fi
    done
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
    unset ROS_LOCALHOST_ONLY
    unset ROS_DISCOVERY_SERVER
    source "${PROJECT_DIR}/install/setup.bash"
fi

# ── 日志 ──────────────────────────────────────────────────────────
LOG_DIR="${PROJECT_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/explore_${PROFILE}_$(date +%Y-%m-%d_%H-%M-%S).log"

echo "============================================="
echo "  自动探索建图场景"
echo "  Profile:     $PROFILE"
echo "  Namespace:   ${NAMESPACE:-无}"
echo "  RViz:        $USE_RVIZ"
echo "  日志:        ${LOG_FILE}"
echo "============================================="

# ── 清理残留 ──────────────────────────────────────────────────────
bash "${PROJECT_DIR}/scripts/tools/cleanup_ros2.sh"

# ── 构建 launch 参数 ──────────────────────────────────────────────
LAUNCH_ARGS="hardware_profile:=${PROFILE} use_rviz:=${USE_RVIZ}"
if [ -n "$NAV2_PARAMS" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS nav2_params_file:=$NAV2_PARAMS"
fi
if [ -n "$SLAM_PARAMS" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS slam_params_file:=$SLAM_PARAMS"
fi
if [ -n "$NAMESPACE" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS namespace:=$NAMESPACE"
fi

# ── 启动 ──────────────────────────────────────────────────────────
exec ros2 launch "${PROJECT_DIR}/launch/explore_main.launch.py" \
    $LAUNCH_ARGS \
    < /dev/null \
    >> "${LOG_FILE}" 2>&1
