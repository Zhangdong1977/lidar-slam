#!/bin/bash
# 仿真环境检查脚本
# 用法: ./scripts/sim_check.sh

# 项目根目录: 优先使用环境变量，否则从脚本位置自动探测
: "${LIDAR_SLAM_ROOT:=$(cd "$(dirname "$0")/.." && pwd)}"

source /opt/ros/jazzy/setup.bash

PASS=0
FAIL=0

check() {
    local desc="$1"
    local cmd="$2"
    echo -n "[$(date +%H:%M:%S)] ${desc} ... "
    if eval "$cmd" > /dev/null 2>&1; then
        echo "OK"
        PASS=$((PASS + 1))
    else
        echo "FAIL"
        FAIL=$((FAIL + 1))
    fi
}

echo "=== 仿真环境检查 ==="

check "Gazebo (gz-sim8)"       "dpkg -s ros-jazzy-gz-sim-vendor"
check "ros_gz_sim"              "dpkg -s ros-jazzy-ros-gz-sim"
check "ros_gz_bridge"           "dpkg -s ros-jazzy-ros-gz-bridge"
check "slam_toolbox"            "dpkg -s ros-jazzy-slam-toolbox"
check "teleop_twist_keyboard"   "dpkg -s ros-jazzy-teleop-twist-keyboard"
check "rviz2"                   "which rviz2"
check "nav2_map_server"         "dpkg -s ros-jazzy-nav2-map-server"

check "机器人模型"               "test -f ${LIDAR_SLAM_ROOT}/models/sim_robot/model.sdf"
check "测试世界"                 "test -f ${LIDAR_SLAM_ROOT}/worlds/slam_test.sdf"
check "SLAM 配置"               "test -f ${LIDAR_SLAM_ROOT}/config/slam_toolbox_sim.yaml"
check "Launch 文件"             "test -f ${LIDAR_SLAM_ROOT}/launch/sim_slam.launch.py"
check "DISPLAY 可用"            "test -n \"\$DISPLAY\" || test -S /tmp/.X11-unix/X0"

echo ""
echo "=== 结果: ${PASS} passed, ${FAIL} failed ==="
