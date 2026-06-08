#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# 远程 RViz 可视化 (Remote RViz Visualization)
#
# 在局域网内另一台机器上运行 RViz, 连接到 Pi 上的 SLAM 节点
# 前提: 两台机器在同一网络, ROS_DOMAIN_ID 一致
#
# 用法:
#   ./scripts/launch/rviz_remote.sh                        # 默认: domain_id=30, slam.rviz
#   ./scripts/launch/rviz_remote.sh --domain-id 30
#   ./scripts/launch/rviz_remote.sh --config /path/to/slam.rviz
#   ./scripts/launch/rviz_remote.sh --profile raspberry    # 自动从 profile 读取 domain_id
#   ./scripts/launch/rviz_remote.sh --config config/nav_multi.rviz  # 多车导航可视化
#   ./scripts/launch/rviz_remote.sh --discovery-address 192.168.10.10
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

DOMAIN_ID=""
RVIZ_CONFIG=""
USE_DISCOVERY_SERVER="True"
DISCOVERY_SERVER_ADDRESS=""
DISCOVERY_SERVER_PORT="11811"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --domain-id)
            DOMAIN_ID="$2"
            shift 2
            ;;
        --config)
            RVIZ_CONFIG="$2"
            shift 2
            ;;
        --profile)
            PROFILE="$2"
            DEFAULT_DOMAIN=$(python3 -c "import yaml; print(yaml.safe_load(open('${PROJECT_DIR}/config/profiles/${PROFILE}.yaml')).get('domain_id', 42))" 2>/dev/null || echo 42)
            DOMAIN_ID="${DOMAIN_ID:-$DEFAULT_DOMAIN}"
            shift 2
            ;;
        --discovery-address)
            DISCOVERY_SERVER_ADDRESS="$2"
            shift 2
            ;;
        --discovery-port)
            DISCOVERY_SERVER_PORT="$2"
            shift 2
            ;;
        --no-discovery-server)
            USE_DISCOVERY_SERVER="False"
            shift
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--domain-id ID] [--config FILE] [--profile NAME] [--discovery-address HOST] [--discovery-port PORT] [--no-discovery-server]"
            exit 1
            ;;
    esac
done

# 默认值
DOMAIN_ID="${DOMAIN_ID:-30}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${PROJECT_DIR}/config/slam.rviz}"

# ROS2 环境
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: ROS2 not found at /opt/ros/jazzy"
    exit 1
fi
source "${PROJECT_DIR}/install/setup.bash" 2>/dev/null || true

export ROS_DOMAIN_ID="$DOMAIN_ID"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY

if [ "$USE_DISCOVERY_SERVER" = "True" ]; then
    if [ -n "$DISCOVERY_SERVER_ADDRESS" ]; then
        if [[ "$DISCOVERY_SERVER_ADDRESS" == *":"* || "$DISCOVERY_SERVER_ADDRESS" == *";"* ]]; then
            export ROS_DISCOVERY_SERVER="$DISCOVERY_SERVER_ADDRESS"
        else
            export ROS_DISCOVERY_SERVER="${DISCOVERY_SERVER_ADDRESS}:${DISCOVERY_SERVER_PORT}"
        fi
    fi
else
    unset ROS_DISCOVERY_SERVER
fi

echo "============================================="
echo "  远程 RViz"
echo "  Domain ID:  $ROS_DOMAIN_ID"
echo "  RMW:        $RMW_IMPLEMENTATION"
echo "  DDS发现:    ${ROS_DISCOVERY_SERVER:-multicast}"
echo "  Config:     $RVIZ_CONFIG"
echo "============================================="
echo ""
echo "提示: 确保远程机器和 Pi 在同一局域网内"
echo "提示: 确保 ROS_DOMAIN_ID 一致 (Pi 上: $ROS_DOMAIN_ID)"
echo ""

exec rviz2 -d "$RVIZ_CONFIG"
