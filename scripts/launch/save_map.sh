#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# 保存地图 (Save Map)
#
# 在本机订阅 slam_toolbox 发布的 /<ns>/map（或根 /map），保存为 PGM + YAML。
# 必须在 SLAM 建图运行中执行；即使 SLAM 跑在远程机器（如树莓派小车），
# 只要本机能收到该 domain 的 /map（与 slam.sh 同 ROS_DOMAIN_ID / 同网段），
# 即可在本机落盘，无需到树莓派上取文件。
#
# 自动匹配 slam.sh 的 profile / namespace / domain_id 设置。
#
# 用法:
#   ./scripts/launch/save_map.sh                                       # 默认: raspberry, maps/map
#   ./scripts/launch/save_map.sh -f maps/my_map                        # 指定文件名 (无扩展名)
#   ./scripts/launch/save_map.sh --namespace c30_1 -f maps/c30_1_map   # 实车 c30_1 (domain 30)
#   ./scripts/launch/save_map.sh --profile gazebo -f maps/sim_map      # 仿真
#   ./scripts/launch/save_map.sh --domain-id 30 -f maps/my_map         # 手动指定 domain_id
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 默认参数
PROFILE="raspberry"
NAMESPACE=""
DOMAIN_ID=""
MAP_NAME="maps/map"

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        -f)
            MAP_NAME="$2"
            shift 2
            ;;
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --namespace)
            NAMESPACE="$2"
            shift 2
            ;;
        --domain-id)
            DOMAIN_ID="$2"
            shift 2
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [-f map_filename] [--profile P] [--namespace NS] [--domain-id ID]"
            exit 1
            ;;
    esac
done

# ── 环境初始化（与 rviz.sh 一致：profile 自动决定 domain_id / use_sim_time）
DEFAULT_DOMAIN=$(python3 -c "import yaml; print(yaml.safe_load(open('${PROJECT_DIR}/config/profiles/${PROFILE}.yaml')).get('domain_id', 42))" 2>/dev/null || echo 42)
USE_SIM_TIME=$(python3 -c "import yaml; print(str(yaml.safe_load(open('${PROJECT_DIR}/config/profiles/${PROFILE}.yaml')).get('use_sim_time', False)).lower())" 2>/dev/null || echo false)
export ROS_DOMAIN_ID="${DOMAIN_ID:-$DEFAULT_DOMAIN}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CycloneDDS 本机发现（allowMulticast=false + loopback peer）
source "${PROJECT_DIR}/scripts/launch/_dds_env.sh"
setup_cyclonedds_uri
trap cyclonedds_cleanup EXIT
unset ROS_LOCALHOST_ONLY
unset ROS_DISCOVERY_SERVER

source /opt/ros/jazzy/setup.bash
source "${PROJECT_DIR}/install/setup.bash" 2>/dev/null || true

# ── 文件名：相对路径相对项目根解析，避免落到意外工作目录 ──────────────
if [[ "$MAP_NAME" != /* ]]; then
    MAP_NAME="${PROJECT_DIR}/${MAP_NAME}"
fi
MAP_DIR="$(dirname "$MAP_NAME")"
mkdir -p "$MAP_DIR"

# ── 构建 ros-args ─────────────────────────────────────────────────
ROS_ARGS=(-p "use_sim_time:=${USE_SIM_TIME}")
if [ -n "$NAMESPACE" ]; then
    # map_saver 只需 __ns：相对名 map 自动解析为 /<ns>/map（与 /map_metadata）
    ROS_ARGS+=(-r "__ns:=/${NAMESPACE}")
fi

# ── 信息打印 ─────────────────────────────────────────────────────
echo "============================================="
echo "  保存地图 (nav2 map_saver)"
echo "  Profile:      $PROFILE"
echo "  Namespace:    ${NAMESPACE:-无}"
echo "  Domain ID:    $ROS_DOMAIN_ID"
echo "  use_sim_time: $USE_SIM_TIME"
echo "  文件:         ${MAP_NAME}.{pgm,yaml}"
echo "============================================="
if [ -n "$NAMESPACE" ]; then
    echo "订阅话题: /${NAMESPACE}/map"
else
    echo "订阅话题: /map"
fi
echo ""
echo "（若长时间无响应，说明本机收不到该话题——检查 domain_id / namespace / 网络互通）"

# ── 保存 ─────────────────────────────────────────────────────────
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" --ros-args "${ROS_ARGS[@]}"

echo "=== 地图已保存 ==="
ls -la "${MAP_NAME}".*
