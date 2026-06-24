#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# Sidecar 端 RViz 启动脚本（由车端 scripts/launch/rviz_sidecar.sh 部署）
#
# 在 sidecar 上经 CycloneDDS unicast 接收车端 Gazebo 仿真话题，
# 启动 TF bridge + rviz2，画面输出到 VNC（默认 :1）。
# 复刻 rviz_multi.sh 的行为，但适配 sidecar 环境（无 conda / 无 install）。
#
# 前提: 本脚本与 rviz_tf_bridge.py / cyclonedds.sidecar.sim.xml /
#       nav_multi.rviz 位于同一目录（由 rviz_sidecar.sh scp 同步）。
#
# 用法（一般由 rviz_sidecar.sh 远程调用，亦可 ssh 进 sidecar 手动运行）:
#   bash _sidecar_rviz.sh
#   bash _sidecar_rviz.sh --namespaces gazebo_1 --domain-id 42
#   bash _sidecar_rviz.sh --display :1 --config nav_multi.rviz
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DOMAIN_ID="42"
NAMESPACES="gazebo_1,gazebo_2"
FIXED_FRAME="map"
RVIZ_CONFIG="${SCRIPT_DIR}/nav_multi.rviz"
DISPLAY_VAL=":1"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --domain-id) DOMAIN_ID="$2"; shift 2 ;;
        --namespaces) NAMESPACES="$2"; shift 2 ;;
        --fixed-frame) FIXED_FRAME="$2"; shift 2 ;;
        --config) RVIZ_CONFIG="$2"; shift 2 ;;
        --display) DISPLAY_VAL="$2"; shift 2 ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--domain-id ID] [--namespaces ns1,ns2] [--fixed-frame FRAME] [--config FILE] [--display :N]"
            exit 1
            ;;
    esac
done

# ROS2 环境（sidecar 已装 jazzy）
if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: ROS2 not found at /opt/ros/jazzy" >&2
    exit 1
fi

export ROS_DOMAIN_ID="$DOMAIN_ID"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_DISCOVERY_SERVER ROS_LOCALHOST_ONLY
# sidecar 静态 cyclonedds 配置: allowMulticast=false, peers=127.0.0.1 + 10.0.0.60
export CYCLONEDDS_URI="file://${SCRIPT_DIR}/cyclonedds.sidecar.sim.xml"

# 输出到 VNC X server
export DISPLAY="$DISPLAY_VAL"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

BRIDGE_PIDS=()
cleanup() {
    for pid in "${BRIDGE_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

IFS=',' read -r -a VEHICLE_NAMESPACES <<< "$NAMESPACES"
for ns in "${VEHICLE_NAMESPACES[@]}"; do
    ns="${ns#/}"
    ns="${ns%/}"
    [ -z "$ns" ] && continue
    # rviz_tf_bridge.py 是纯 rclpy 脚本，无需 install；source jazzy 后可直接跑
    python3 "${SCRIPT_DIR}/rviz_tf_bridge.py" \
        --ros-args \
        -r "__node:=rviz_tf_bridge_${ns}" \
        -p "namespace:=${ns}" \
        -p "fixed_frame:=${FIXED_FRAME}" &
    BRIDGE_PIDS+=("$!")
done

echo "============================================="
echo "  Sidecar 远程 RViz"
echo "  Domain ID:  $ROS_DOMAIN_ID"
echo "  RMW:        $RMW_IMPLEMENTATION"
echo "  DDS配置:    $CYCLONEDDS_URI"
echo "  DISPLAY:    $DISPLAY"
echo "  Namespaces: $NAMESPACES"
echo "  Config:     $RVIZ_CONFIG"
echo "  TF bridge:  ${#BRIDGE_PIDS[@]} 个 (fixed frame: $FIXED_FRAME)"
echo "============================================="
echo "RViz scan topics: /rviz/<namespace>/scan"
echo ""

# rviz2 前台运行；退出时由 trap 清理 TF bridge
rviz2 -d "$RVIZ_CONFIG"
