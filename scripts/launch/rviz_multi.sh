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
CYCLONE_PEERS=()

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
            CYCLONE_PEERS+=("$2")
            shift 2
            ;;
        --peer)
            CYCLONE_PEERS+=("$2")
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
            echo "Usage: $0 [--domain-id ID] [--profile gazebo|raspberry|rs485] [--config FILE] [--namespaces ns1,ns2] [--fixed-frame FRAME] [--no-tf-bridge] [--discovery-address HOST] [--peer HOST] [--no-discovery-server]"
            exit 1
            ;;
    esac
done

DEFAULT_DOMAIN=$(python3 -c "import yaml; print(yaml.safe_load(open('${PROJECT_DIR}/config/profiles/${PROFILE}.yaml')).get('domain_id', 42))" 2>/dev/null || echo 42)
export ROS_DOMAIN_ID="${DOMAIN_ID:-$DEFAULT_DOMAIN}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY

# CycloneDDS unicast 发现配置（替代 FastDDS Discovery Server）
source "${PROJECT_DIR}/scripts/launch/_dds_env.sh"

if [ "$USE_DISCOVERY_SERVER" = "True" ] && [ ${#CYCLONE_PEERS[@]} -gt 0 ]; then
    setup_cyclonedds_uri "${CYCLONE_PEERS[@]}"
elif [ "$USE_DISCOVERY_SERVER" = "True" ]; then
    echo "WARNING: CycloneDDS unicast 已启用但未指定 peer (--discovery-address)。"
    echo "         将使用本机 loopback 发现；若 dispatch 在其它主机，RViz 将无法发现。"
    echo "         请使用 --discovery-address <sidecar_or_vehicle_ip> 或 --no-discovery-server。"
    echo ""
else
    unset ROS_DISCOVERY_SERVER
    unset CYCLONEDDS_URI
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
    cyclonedds_cleanup
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
echo "  DDS发现:    CycloneDDS peers=[${CYCLONE_PEERS[*]:-本机loopback}] (禁 multicast)"
echo "  Config:     $RVIZ_CONFIG"
echo "  Namespaces: $NAMESPACES"
echo "  TF bridge:  $USE_TF_BRIDGE (fixed frame: $FIXED_FRAME)"
echo "============================================="
echo "RViz scan topics: /rviz/<namespace>/scan"
echo ""

rviz2 -d "$RVIZ_CONFIG"
