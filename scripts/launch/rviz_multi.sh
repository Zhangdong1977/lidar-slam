#!/bin/bash
# Multi-vehicle RViz for Gazebo namespace-isolated simulation.
#
# Usage:
#   ./scripts/launch/rviz_multi.sh
#   ./scripts/launch/rviz_multi.sh --domain-id 42
#   ./scripts/launch/rviz_multi.sh --config config/nav_multi.rviz

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

DOMAIN_ID=""
RVIZ_CONFIG="${PROJECT_DIR}/config/nav_multi.rviz"

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
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [--domain-id ID] [--config FILE]"
            exit 1
            ;;
    esac
done

DEFAULT_DOMAIN=$(python3 -c "import yaml; print(yaml.safe_load(open('${PROJECT_DIR}/config/profiles/gazebo.yaml')).get('domain_id', 42))" 2>/dev/null || echo 42)
export ROS_DOMAIN_ID="${DOMAIN_ID:-$DEFAULT_DOMAIN}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
unset ROS_DISCOVERY_SERVER

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: ROS2 not found at /opt/ros/jazzy"
    exit 1
fi
source "${PROJECT_DIR}/install/setup.bash" 2>/dev/null || true

echo "============================================="
echo "  Multi-vehicle RViz"
echo "  Domain ID:  $ROS_DOMAIN_ID"
echo "  Config:     $RVIZ_CONFIG"
echo "============================================="
echo "Watching default namespaces: /gazebo_1 and /gazebo_2"
echo ""

exec rviz2 -d "$RVIZ_CONFIG"
