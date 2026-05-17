#!/bin/bash
set -e

echo "=== Cleaning up ROS2, Gazebo, and RViz ==="

# Kill ROS2 nodes and daemon
echo -n "Stopping ROS2 daemon... "
ros2 daemon stop 2>/dev/null || killall -q ros2_daemon 2>/dev/null || true
echo "done"

echo -n "Killing ROS2 nodes... "
killall -q -9 ros2 2>/dev/null || true
echo "done"

# Kill Gazebo
echo -n "Killing Gazebo... "
killall -q -9 gzserver gzclient gazebo 2>/dev/null || true
echo "done"

# Kill RViz
echo -n "Killing RViz... "
killall -q -9 rviz2 2>/dev/null || true
echo "done"

# Kill lingering ROS tools
for proc in robot_state_publisher joint_state_publisher controller_manager rqt; do
    killall -q -9 "$proc" 2>/dev/null || true
done

# Clean shared memory if ipcs is available
if command -v ipcs &>/dev/null && command -v ipcrm &>/dev/null; then
    echo -n "Cleaning orphaned shared memory... "
    ipcs -m 2>/dev/null | awk '/0x[0-9a-f]+/ {print $2}' | xargs -r -n1 ipcrm -m 2>/dev/null || true
    echo "done"
fi

echo "=== Cleanup complete ==="
