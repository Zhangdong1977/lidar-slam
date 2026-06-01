#!/bin/bash
# Launch navigation with RS-485 physical chassis
# RPLIDAR S2L + car_base_node (odom+IMU) + RS-485 chassis bridge
#
# Prerequisites:
#   conda activate lidar_slam
#   RS-485 adapter on /dev/ttyUSB1
#   RPLIDAR on /dev/ttyUSB0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Environment
eval "$(conda shell.bash hook)"
conda activate lidar_slam
export LIDAR_SLAM_ROOT="${LIDAR_SLAM_ROOT:-$PROJECT_DIR}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER

source /opt/ros/jazzy/setup.bash
source "${LIDAR_SLAM_ROOT}/install/setup.bash"

# Cleanup stale processes
bash "${SCRIPT_DIR}/../tools/cleanup_ros2.sh" 2>/dev/null || true

# Map file
MAP_FILE="${1:-${LIDAR_SLAM_ROOT}/maps/auto_exploration_map.yaml}"
if [ ! -f "$MAP_FILE" ]; then
    echo "ERROR: Map file not found: $MAP_FILE"
    echo "Usage: $0 [map_file.yaml]"
    exit 1
fi

echo "=== RS-485 Physical Chassis Navigation ==="
echo "Profile: rs485"
echo "Map: $MAP_FILE"

ros2 launch "${LIDAR_SLAM_ROOT}/launch/nav_main.launch.py" \
    hardware_profile:=rs485 \
    use_respawn:=True \
    vehicle_name:=ackermann_robot \
    map_file:="$MAP_FILE" \
    2>&1 | tee "${LIDAR_SLAM_ROOT}/log/rs485_nav_$(date +%Y-%m-%d_%H-%M-%S).log"
