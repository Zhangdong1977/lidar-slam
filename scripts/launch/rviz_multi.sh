#!/bin/bash
# Multi-vehicle RViz for Gazebo namespace-isolated simulation.
#
# Usage:
#   ./scripts/launch/rviz_multi.sh
#   ./scripts/launch/rviz_multi.sh --domain-id 42
#   ./scripts/launch/rviz_multi.sh --config config/nav_multi.rviz
#   ./scripts/launch/rviz_multi.sh --discovery-address 10.0.0.187
#   ./scripts/launch/rviz_multi.sh --namespaces gazebo_1,gazebo_2

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

DOMAIN_ID=""
PROFILE="gazebo"
RVIZ_CONFIG="${PROJECT_DIR}/config/nav_multi.rviz"
NAMESPACES="gazebo_1,gazebo_2"
FIXED_FRAME="map"
USE_TF_BRIDGE="True"
USE_DISCOVERY_SERVER="True"
DISCOVERY_SERVER_ADDRESS=""
DISCOVERY_SERVER_PORT="11811"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --domain-id)
            DOMAIN_ID="$2"
            shift 2
            ;;
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --config)
            RVIZ_CONFIG="$2"
            shift 2
            ;;
        --namespaces)
            NAMESPACES="$2"
            shift 2
            ;;
        --fixed-frame)
            FIXED_FRAME="$2"
            shift 2
            ;;
        --no-tf-bridge)
            USE_TF_BRIDGE="False"
            shift
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
            echo "Unknown argument: $1"
            echo "Usage: $0 [--domain-id ID] [--profile gazebo|raspberry|rs485] [--config FILE] [--namespaces ns1,ns2] [--fixed-frame FRAME] [--no-tf-bridge] [--discovery-address HOST] [--discovery-port PORT] [--no-discovery-server]"
            exit 1
            ;;
    esac
done

DEFAULT_DOMAIN=$(python3 -c "import yaml; print(yaml.safe_load(open('${PROJECT_DIR}/config/profiles/${PROFILE}.yaml')).get('domain_id', 42))" 2>/dev/null || echo 42)
export ROS_DOMAIN_ID="${DOMAIN_ID:-$DEFAULT_DOMAIN}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY

if [ "$USE_DISCOVERY_SERVER" = "True" ]; then
    if [ -n "$DISCOVERY_SERVER_ADDRESS" ]; then
        if [[ "$DISCOVERY_SERVER_ADDRESS" == *":"* || "$DISCOVERY_SERVER_ADDRESS" == *";"* ]]; then
            export ROS_DISCOVERY_SERVER="$DISCOVERY_SERVER_ADDRESS"
        else
            export ROS_DISCOVERY_SERVER="${DISCOVERY_SERVER_ADDRESS}:${DISCOVERY_SERVER_PORT}"
        fi
    elif [ -z "${ROS_DISCOVERY_SERVER:-}" ]; then
        echo "WARNING: Discovery Server 已启用但未指定地址 (--discovery-address)。"
        echo "         未设置 ROS_DISCOVERY_SERVER，将使用 multicast 发现。"
        echo "         如果 dispatch 节点连接了 Discovery Server，RViz 将无法发现它们。"
        echo "         请使用 --discovery-address <sidecar_ip> 或 --no-discovery-server。"
        echo ""
    fi
else
    unset ROS_DISCOVERY_SERVER
fi

export DISPLAY=${DISPLAY:-:0}
for auth in /run/user/$(id -u)/.mutter-Xwaylandauth.* /home/$(whoami)/.Xauthority; do
    if [ -f "$auth" ]; then
        export XAUTHORITY="$auth"
        break
    fi
done

eval "$(conda shell.bash hook 2>/dev/null)" || true
conda activate lidar_slam 2>/dev/null || true

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: ROS2 not found at /opt/ros/jazzy"
    exit 1
fi
source "${PROJECT_DIR}/install/setup.bash" 2>/dev/null || true

IFS=',' read -r -a VEHICLE_NAMESPACES <<< "$NAMESPACES"
BRIDGE_PIDS=()

cleanup() {
    for pid in "${BRIDGE_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT INT TERM

start_rviz_bridge() {
    local ns="$1"
    ns="${ns#/}"
    ns="${ns%/}"
    if [ -z "$ns" ]; then
        return
    fi

    if ros2 pkg executables lidar_slam_nodes 2>/dev/null | awk '{print $2}' | grep -qx rviz_tf_bridge; then
        ros2 run lidar_slam_nodes rviz_tf_bridge \
            --ros-args \
            -r "__node:=rviz_tf_bridge_${ns}" \
            -p "namespace:=${ns}" \
            -p "fixed_frame:=${FIXED_FRAME}" &
    else
        python3 "${PROJECT_DIR}/src/lidar_slam_nodes/lidar_slam_nodes/rviz_tf_bridge.py" \
            --ros-args \
            -r "__node:=rviz_tf_bridge_${ns}" \
            -p "namespace:=${ns}" \
            -p "fixed_frame:=${FIXED_FRAME}" &
    fi
    BRIDGE_PIDS+=("$!")
}

if [ "$USE_TF_BRIDGE" = "True" ]; then
    for ns in "${VEHICLE_NAMESPACES[@]}"; do
        start_rviz_bridge "$ns"
    done
fi

echo "============================================="
echo "  Multi-vehicle RViz"
echo "  Profile:    $PROFILE"
echo "  Domain ID:  $ROS_DOMAIN_ID"
echo "  DDS发现:    ${ROS_DISCOVERY_SERVER:-multicast}"
echo "  Config:     $RVIZ_CONFIG"
echo "  Namespaces: $NAMESPACES"
echo "  TF bridge:  $USE_TF_BRIDGE (fixed frame: $FIXED_FRAME)"
echo "============================================="
echo "RViz scan topics: /rviz/<namespace>/scan"
echo ""

rviz2 -d "$RVIZ_CONFIG"
