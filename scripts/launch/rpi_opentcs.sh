#!/bin/bash
# Launch navigation on Raspberry Pi car (hardware_profile=raspberry)
# car_base_node drives STM32 via UART, RPLIDAR C1 provides laser scan
#
# Prerequisites:
#   source /opt/ros/jazzy/setup.bash
#   source /home/pi/ros2_ws/install/setup.bash   (car_base_node)
#   source /home/pi/jvs/install/setup.bash         (lidar_slam_nodes)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Environment
export LIDAR_SLAM_ROOT="${LIDAR_SLAM_ROOT:-$PROJECT_DIR}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
unset ROS_DISCOVERY_SERVER

# Source workspaces (order matters: overlay last)
source /opt/ros/jazzy/setup.bash
source /home/pi/ros2_ws/install/setup.bash 2>/dev/null || true
source "${LIDAR_SLAM_ROOT}/install/setup.bash"

# Map file (required argument or default)
MAP_FILE="${1:-${LIDAR_SLAM_ROOT}/maps/auto_exploration_map.yaml}"
if [ ! -f "$MAP_FILE" ]; then
    echo "ERROR: Map file not found: $MAP_FILE"
    echo "Usage: $0 [map_file.yaml]"
    exit 1
fi

echo "=== Raspberry Pi Car Navigation ==="
echo "Profile: raspberry"
echo "Map: $MAP_FILE"
echo "LIDAR_SLAM_ROOT: $LIDAR_SLAM_ROOT"

ros2 launch "${LIDAR_SLAM_ROOT}/launch/nav_main.launch.py" \
    hardware_profile:=raspberry \
    use_respawn:=True \
    vehicle_name:=raspberry_agv \
    map_file:="$MAP_FILE" \
    2>&1 | tee "${LIDAR_SLAM_ROOT}/log/rpi_nav_$(date +%Y-%m-%d_%H-%M-%S).log"
