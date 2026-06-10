#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# 场景1: 手工建图 (Manual SLAM Mapping)
#
# 启动硬件层 + 传感器融合 + slam_toolbox 在线建图 + 键盘遥控
# 数据流: 传感器 → EKF/rf2o → slam_toolbox → /map + TF map→odom
#         teleop 键盘 → /cmd_vel → 底盘
#
# 用法:
#   ./scripts/launch/slam.sh                                # 默认: gazebo 仿真, 无键盘遥控
#   ./scripts/launch/slam.sh --profile rs485                # RS-485 实车
#   ./scripts/launch/slam.sh --profile raspberry            # 树莓派
#   ./scripts/launch/slam.sh --profile rplidar_s2l          # 纯激光雷达 (无底盘)
#   ./scripts/launch/slam.sh --joystick                     # 启动手柄遥控 (发布 /cmd_vel)
#   ./scripts/launch/slam.sh --no-teleop                    # 不启动键盘遥控
#   ./scripts/launch/slam.sh --no-rviz                      # 不启动 RViz (用于远程可视化)
#   ./scripts/launch/slam.sh --slam-params /path/to/params  # 自定义SLAM参数
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 默认参数
PROFILE="gazebo"
USE_TELEOP="False"
USE_JOYSTICK="False"
USE_RVIZ="True"
SLAM_PARAMS=""
EXTRA_ARGS=""
DOMAIN_ID=""
NAMESPACE=""

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --teleop)
            USE_TELEOP="True"
            shift
            ;;
        --joystick)
            USE_JOYSTICK="True"
            shift
            ;;
        --no-rviz)
            USE_RVIZ="False"
            shift
            ;;
        --no-teleop)
            USE_TELEOP="False"
            shift
            ;;
        --slam-params)
            SLAM_PARAMS="$2"
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
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--profile gazebo|rs485|raspberry|rplidar_s2l] [--teleop] [--joystick] [--no-teleop] [--no-rviz] [--slam-params FILE] [--domain-id ID] [--namespace NS]"
            exit 1
            ;;
    esac
done

# ── 环境初始化 (按 profile 区分) ──────────────────────────────────
# 从 profile YAML 读取 domain_id（默认 42）
DEFAULT_DOMAIN=$(python3 -c "import yaml; print(yaml.safe_load(open('${PROJECT_DIR}/config/profiles/${PROFILE}.yaml')).get('domain_id', 42))" 2>/dev/null || echo 42)
export ROS_DOMAIN_ID="${DOMAIN_ID:-$DEFAULT_DOMAIN}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CycloneDDS 本机发现（allowMulticast=false + loopback peer；跨机由对端配 peer 主动连接）
source "${PROJECT_DIR}/scripts/launch/_dds_env.sh"
setup_cyclonedds_uri
trap cyclonedds_cleanup EXIT

if [ "$PROFILE" = "raspberry" ]; then
    unset ROS_DISCOVERY_SERVER
    source /opt/ros/jazzy/setup.bash
    source /home/pi/ros2_ws/install/setup.bash 2>/dev/null || true
    source "${PROJECT_DIR}/install/setup.bash"

    # 串口检查
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
    # 仿真/实车环境: conda + fastrtps
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

# ── 手柄设备检测 ──────────────────────────────────────────────────
if [ "$USE_JOYSTICK" = "True" ] && [ ! -e /dev/input/js0 ]; then
    echo "WARN: 游戏手柄未连接 (/dev/input/js0), 跳过手柄控制"
    USE_JOYSTICK="False"
fi

# ── 日志 ──────────────────────────────────────────────────────────
LOG_DIR="${PROJECT_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/slam_${PROFILE}_$(date +%Y-%m-%d_%H-%M-%S).log"

echo "============================================="
echo "  手工建图场景"
echo "  Profile:     $PROFILE"
echo "  Namespace:   ${NAMESPACE:-无}"
echo "  Teleop:      $USE_TELEOP"
echo "  Joystick:    $USE_JOYSTICK"
echo "  RViz:        $USE_RVIZ"
if [ -n "$SLAM_PARAMS" ]; then
    echo "  SLAM参数: $SLAM_PARAMS"
fi
echo "  日志:    ${LOG_FILE}"
echo "============================================="

# ── 清理残留 ──────────────────────────────────────────────────────
bash "${PROJECT_DIR}/scripts/tools/cleanup_ros2.sh"

# ── 构建 launch 参数 ──────────────────────────────────────────────
LAUNCH_ARGS="hardware_profile:=${PROFILE} use_teleop:=${USE_TELEOP} use_joystick:=${USE_JOYSTICK} use_rviz:=${USE_RVIZ}"
if [ -n "$SLAM_PARAMS" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS slam_params_file:=$SLAM_PARAMS"
fi
if [ -n "$NAMESPACE" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS namespace:=$NAMESPACE"
fi

# ── 启动 ──────────────────────────────────────────────────────────
# 键盘遥控需要 stdin 是 TTY, 且 stdout 在终端可见;
# 非遥控模式则全部重定向到日志, 后台干净运行.
if [ "$USE_TELEOP" = "True" ]; then
    exec ros2 launch "${PROJECT_DIR}/launch/slam_main.launch.py" \
        $LAUNCH_ARGS \
        2>&1 | tee "${LOG_FILE}"
else
    exec ros2 launch "${PROJECT_DIR}/launch/slam_main.launch.py" \
        $LAUNCH_ARGS \
        < /dev/null \
        >> "${LOG_FILE}" 2>&1
fi
