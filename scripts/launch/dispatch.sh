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
#   ./scripts/launch/dispatch.sh                                 # 默认: gazebo 仿真 (gazebo_1)
#   ./scripts/launch/dispatch.sh --namespace gazebo_2            # 仿真第二辆车
#   ./scripts/launch/dispatch.sh --namespace gazebo_2 --spawn-y 2.0  # 指定 Gazebo 初始位姿
#   ./scripts/launch/dispatch.sh --profile raspberry --namespace c30_1  # 树莓派实车
#   ./scripts/launch/dispatch.sh --profile rs485                 # RS-485 实车
#   ./scripts/launch/dispatch.sh --map /path/to/map.yaml         # 指定地图
#   ./scripts/launch/dispatch.sh --no-rviz                      # 不启动 RViz
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 默认参数
PROFILE="gazebo"
MAP_FILE="${PROJECT_DIR}/maps/auto_exploration_map.yaml"
DOMAIN_ID=""
NAMESPACE="gazebo_1"
USE_RVIZ="True"
SPAWN_X=""
SPAWN_Y=""
SPAWN_Z="0.24"
INITIAL_POSE_X=""
INITIAL_POSE_Y=""
INITIAL_POSE_YAW="0.0"
SKIP_CLEANUP="False"
START_GAZEBO="True"

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
        --domain-id)
            DOMAIN_ID="$2"
            shift 2
            ;;
        --namespace)
            NAMESPACE="$2"
            shift 2
            ;;
        --spawn-x)
            SPAWN_X="$2"
            shift 2
            ;;
        --spawn-y)
            SPAWN_Y="$2"
            shift 2
            ;;
        --spawn-z)
            SPAWN_Z="$2"
            shift 2
            ;;
        --initial-x)
            INITIAL_POSE_X="$2"
            shift 2
            ;;
        --initial-y)
            INITIAL_POSE_Y="$2"
            shift 2
            ;;
        --initial-yaw)
            INITIAL_POSE_YAW="$2"
            shift 2
            ;;
        --no-rviz)
            USE_RVIZ="False"
            shift
            ;;
        --skip-cleanup)
            SKIP_CLEANUP="True"
            shift
            ;;
        --no-gazebo)
            START_GAZEBO="False"
            shift
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--profile gazebo|rs485|raspberry] [--namespace NAME] [--spawn-x X] [--spawn-y Y] [--spawn-z Z] [--initial-x X] [--initial-y Y] [--initial-yaw YAW] [--map FILE] [--no-rviz] [--skip-cleanup] [--no-gazebo] [--domain-id ID]"
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

# ── 地图检查 ──────────────────────────────────────────────────────
if [ ! -f "$MAP_FILE" ]; then
    echo "ERROR: 地图文件不存在: $MAP_FILE"
    echo "用法: $0 [--map FILE]"
    exit 1
fi

# ── Gazebo 多车初始位姿 ───────────────────────────────────────────
# 默认规则: gazebo_1 -> (0, 0), gazebo_2 -> (0, 2), gazebo_N -> (0, 2*(N-1)).
# 可通过 --spawn-x/--spawn-y 显式覆盖。AMCL 初始位姿默认跟随 Gazebo spawn。
if [ "$PROFILE" = "gazebo" ]; then
    if [ -z "$SPAWN_X" ]; then
        SPAWN_X="0"
    fi
    if [ -z "$SPAWN_Y" ]; then
        if [[ "$NAMESPACE" =~ ^gazebo_([0-9]+)$ ]]; then
            vehicle_index=$((10#${BASH_REMATCH[1]}))
            SPAWN_Y="$(( (vehicle_index - 1) * 2 ))"
        else
            SPAWN_Y="0"
        fi
    fi
fi

as_float_literal() {
    if [[ "$1" =~ ^-?[0-9]+$ ]]; then
        echo "$1.0"
    else
        echo "$1"
    fi
}

if [ -z "$INITIAL_POSE_X" ]; then
    INITIAL_POSE_X="${SPAWN_X:-0.0}"
fi
if [ -z "$INITIAL_POSE_Y" ]; then
    INITIAL_POSE_Y="${SPAWN_Y:-0.0}"
fi
INITIAL_POSE_X="$(as_float_literal "$INITIAL_POSE_X")"
INITIAL_POSE_Y="$(as_float_literal "$INITIAL_POSE_Y")"
INITIAL_POSE_YAW="$(as_float_literal "$INITIAL_POSE_YAW")"

# ── 日志 ──────────────────────────────────────────────────────────
LOG_DIR="${PROJECT_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/dispatch_${PROFILE}_$(date +%Y-%m-%d_%H-%M-%S).log"

echo "============================================="
echo "  调度集成场景"
echo "  Profile:   $PROFILE"
echo "  Namespace: ${NAMESPACE:-'(none)'}"
echo "  Spawn:     x=${SPAWN_X:-'(default)'} y=${SPAWN_Y:-'(default)'} z=${SPAWN_Z:-'(default)'}"
echo "  AMCL init: x=${INITIAL_POSE_X} y=${INITIAL_POSE_Y} yaw=${INITIAL_POSE_YAW}"
echo "  RViz:      $USE_RVIZ"
echo "  Domain:    $ROS_DOMAIN_ID"
echo "  地图:      $MAP_FILE"
echo "  日志:      ${LOG_FILE}"
echo "============================================="

# ── 清理残留 ──────────────────────────────────────────────────────
if [ "$SKIP_CLEANUP" = "True" ]; then
    echo "跳过 ROS2/Gazebo 清理 (--skip-cleanup)"
else
    bash "${PROJECT_DIR}/scripts/tools/cleanup_ros2.sh"
fi

# ── 启动 ──────────────────────────────────────────────────────────
exec ros2 launch "${PROJECT_DIR}/launch/nav_main.launch.py" \
    hardware_profile:="${PROFILE}" \
    use_respawn:=True \
    use_rviz:="${USE_RVIZ}" \
    vehicle_name:="${NAMESPACE}" \
    namespace:="${NAMESPACE}" \
    map_file:="$MAP_FILE" \
    spawn_x:="$SPAWN_X" \
    spawn_y:="$SPAWN_Y" \
    spawn_z:="$SPAWN_Z" \
    initial_pose_x:="$INITIAL_POSE_X" \
    initial_pose_y:="$INITIAL_POSE_Y" \
    initial_pose_yaw:="$INITIAL_POSE_YAW" \
    start_gazebo:="$START_GAZEBO" \
    < /dev/null \
    >> "${LOG_FILE}" 2>&1
