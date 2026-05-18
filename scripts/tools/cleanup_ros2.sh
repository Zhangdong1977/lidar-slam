#!/bin/bash
set -e

WORKSPACE=${1:-"lidar-slam"}
echo "=== Cleaning up ROS2, Gazebo, and RViz (workspace: $WORKSPACE) ==="

# ── Phase 1: graceful shutdown (SIGTERM) ──────────────────────────
echo -n "Sending SIGTERM to all ROS/Gazebo processes... "

# ROS2 daemon
pkill -f "ros2-daemon" 2>/dev/null || true
pkill -f "ros2cli.daemon" 2>/dev/null || true

# Workspace nodes (matched by install path)
pkill -f "${WORKSPACE}/install/" 2>/dev/null || true

# ros_gz_bridge (system-installed)
pkill -f "ros_gz_bridge" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true

# System ROS nodes — use pkill to avoid killall's 15-char name limit
pkill -f "static_transform_publisher" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "joint_state_publisher" 2>/dev/null || true
pkill -f "controller_manager" 2>/dev/null || true
pkill -f "rqt" 2>/dev/null || true

# Gazebo classic + modern (gz sim)
pkill -f "gzserver" 2>/dev/null || true
pkill -f "gzclient" 2>/dev/null || true
pkill -f "gazebo" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true

# RViz2
pkill -f "rviz2" 2>/dev/null || true

echo "done"

# Give processes a moment to exit
sleep 0.8

# ── Phase 2: force kill anything still alive (SIGKILL) ────────────
echo -n "Force-killing survivors... "

pkill -9 -f "ros2-daemon" 2>/dev/null || true
pkill -9 -f "ros2cli.daemon" 2>/dev/null || true
pkill -9 -f "${WORKSPACE}/install/" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "static_transform_publisher" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "joint_state_publisher" 2>/dev/null || true
pkill -9 -f "controller_manager" 2>/dev/null || true
pkill -9 -f "rqt" 2>/dev/null || true
pkill -9 -f "gzserver" 2>/dev/null || true
pkill -9 -f "gzclient" 2>/dev/null || true
pkill -9 -f "gazebo" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true

echo "done"

# ── Temp files ────────────────────────────────────────────────────
echo -n "Cleaning ROS2 temp files... "
rm -f /tmp/ros2_daemon_* 2>/dev/null || true
rm -rf /tmp/.ros/ 2>/dev/null || true
echo "done"

# ── Fast-DDS shared memory (file-based, ROS2 Humble+) ─────────────
echo -n "Cleaning Fast-DDS shared memory... "
rm -f /dev/shm/fastrtps_* /dev/shm/*ros* 2>/dev/null || true
echo "done"

# ── SysV shared memory (current user only) ────────────────────────
if command -v ipcs &>/dev/null && command -v ipcrm &>/dev/null; then
    echo -n "Cleaning orphaned SysV shared memory... "
    ipcs -m 2>/dev/null | awk -v uid="$(id -u)" '$3 == uid {print $2}' \
        | xargs -r -n1 ipcrm -m 2>/dev/null || true
    echo "done"
fi

echo "=== Cleanup complete ==="
