#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# 本地 RViz 可视化 (配套 slam.sh 使用)
#
# 当 slam.sh 以 --no-rviz 启动后，用此脚本单独启动 RViz。
# 自动匹配 slam.sh 的 profile / namespace / domain_id 设置，
# 并将 RViz 放入正确的命名空间以订阅 /<ns>/map, /<ns>/scan 等。
#
# 用法:
#   ./scripts/launch/rviz.sh                              # 默认: raspberry, namespace=c30_1
#   ./scripts/launch/rviz.sh --profile raspberry
#   ./scripts/launch/rviz.sh --profile raspberry --namespace c30_1
#   ./scripts/launch/rviz.sh --namespace c30_1            # 自动从 profile 读取 domain_id
#   ./scripts/launch/rviz.sh --config config/explore.rviz # 自定义 rviz 配置
#   ./scripts/launch/rviz.sh --domain-id 30               # 手动指定 domain_id
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 默认参数
PROFILE="raspberry"
NAMESPACE=""
DOMAIN_ID=""
RVIZ_CONFIG=""

while [[ $# -gt 0 ]]; do
    case "$1" in
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
        --config)
            RVIZ_CONFIG="$2"
            shift 2
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--profile gazebo|raspberry|rs485] [--namespace NS] [--domain-id ID] [--config FILE]"
            exit 1
            ;;
    esac
done

# ── 环境初始化 ──────────────────────────────────────────────────
DEFAULT_DOMAIN=$(python3 -c "import yaml; print(yaml.safe_load(open('${PROJECT_DIR}/config/profiles/${PROFILE}.yaml')).get('domain_id', 42))" 2>/dev/null || echo 42)
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

# ── 自动选择 rviz 配置 ────────────────────────────────────────
if [ -z "$RVIZ_CONFIG" ]; then
    case "$PROFILE" in
        explore|navigation)
            RVIZ_CONFIG="${PROJECT_DIR}/config/explore.rviz"
            ;;
        *)
            RVIZ_CONFIG="${PROJECT_DIR}/config/slam.rviz"
            ;;
    esac
fi

# ── 构建 ros-args ─────────────────────────────────────────────
ROS_ARGS=()
if [ -n "$NAMESPACE" ]; then
    ROS_ARGS+=(-r "__ns:=/${NAMESPACE}")
    # rviz 内部 tf2_ros::TransformListener 创建的节点用绝对路径 /tf, /tf_static,
    # 不遵守 __ns 命名空间。必须显式 remap 到相对路径，才能解析到 /<ns>/tf。
    ROS_ARGS+=(-r "/tf:=tf" -r "/tf_static:=tf_static")
fi
ROS_ARGS+=(-p "use_sim_time:=false")

echo "============================================="
echo "  RViz 可视化 (配套 slam.sh)"
echo "  Profile:    $PROFILE"
echo "  Namespace:  ${NAMESPACE:-无}"
echo "  Domain ID:  $ROS_DOMAIN_ID"
echo "  Config:     $RVIZ_CONFIG"
echo "============================================="

if [ -n "$NAMESPACE" ]; then
    echo "话题: /${NAMESPACE}/map, /${NAMESPACE}/scan, /${NAMESPACE}/tf"
else
    echo "话题: /map, /scan, /tf"
fi
echo ""

exec rviz2 -d "$RVIZ_CONFIG" --ros-args "${ROS_ARGS[@]}"
