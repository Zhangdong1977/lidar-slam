#!/bin/bash
# 树莓派小车自动探索建图脚本
# 启动 RPLIDAR C1 + car_base_node (STM32) + slam_toolbox + frontier_explorer 自动探索
#
# 场景定位: 真实小车自主探索建图
# 数据流: 传感器 → EKF + slam_toolbox
#         frontier_explorer 选择 frontier → NavigateToPose → Nav2 → /cmd_vel
#         car_base_node 订阅 /cmd_vel → STM32 UART → 电机
# 适合: 未知环境自动建图
#
# 注意: 树莓派资源紧张,建议使用低频率 (Nav2 周期 ≥ 0.2s, 控制频率 ≤ 10Hz)
#
# 硬件要求:
#   RPLIDAR C1: /dev/lidar (460800 baud)
#   STM32:      /dev/ttyAMA0 (115200 baud)
#
# 用法:
#   ./scripts/launch/rpi_explore.sh
#
# 配套: 另开终端保存地图
#   ros2 run nav2_map_server map_saver_cli -f ~/maps/raspberry_auto_map

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# ROS2 环境
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER

source /opt/ros/jazzy/setup.bash
source /home/pi/ros2_ws/install/setup.bash 2>/dev/null || true
source "${PROJECT_DIR}/install/setup.bash"

# 串口设备检查
LIDAR_PORT="${LIDAR_PORT:-/dev/lidar}"
CHASSIS_PORT="${CHASSIS_PORT:-/dev/ttyAMA0}"
for dev in "$LIDAR_PORT" "$CHASSIS_PORT"; do
    if [ ! -e "$dev" ]; then
        echo "ERROR: 串口设备不存在: $dev"
        exit 1
    fi
    if [ ! -r "$dev" ] || [ ! -w "$dev" ]; then
        sudo chmod 666 "$dev" 2>/dev/null || \
            echo "WARN: 请手动执行 sudo chmod 666 $dev"
    fi
done

# 日志输出到 log 目录
LOG_DIR="${PROJECT_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/rpi_explore_$(date +%Y-%m-%d_%H-%M-%S).log"

SLAM_PARAMS="${PROJECT_DIR}/config/slam_toolbox_real.yaml"
NAV2_PARAMS="${PROJECT_DIR}/config/nav2_params_exploration.yaml"
FRONTIER_PARAMS="${PROJECT_DIR}/config/frontier_explorer_params.yaml"
RVIZ_CONFIG="${PROJECT_DIR}/config/explore.rviz"

echo "============================================="
echo "  树莓派小车自动探索建图场景"
echo "  Profile:  raspberry (实车)"
echo "  雷达:     $LIDAR_PORT (RPLIDAR C1)"
echo "  底盘:     $CHASSIS_PORT (STM32)"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  日志:     ${LOG_FILE}"
echo "============================================="

# 1. RPLIDAR C1
ros2 run rplidar_ros rplidar_node \
    --ros-args \
    -p channel_type:=serial \
    -p serial_port:="$LIDAR_PORT" \
    -p serial_baudrate:=460800 \
    -p frame_id:=lidar_link \
    -p inverted:=false \
    -p angle_compensate:=true \
    -p scan_mode:=Sensitivity \
    -p use_sim_time:=false \
    -r __node:=rplidar_node \
    >> "${LOG_FILE}" 2>&1 &
LIDAR_PID=$!

# 2. car_base_node
ros2 run car_base car_base_node \
    --ros-args \
    -p odom_frame_id:=odom \
    -p robot_frame_id:=base_link \
    -p gyro_frame_id:=imu_link \
    -p use_sim_time:=false \
    -r __node:=car_base_node \
    >> "${LOG_FILE}" 2>&1 &
CAR_PID=$!

# 3. static TF: base_link -> lidar_link
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0.15 \
    --roll 0 --pitch 0 --yaw 3.14159 \
    --frame-id base_link --child-frame-id lidar_link \
    --ros-args -p use_sim_time:=false \
    >> "${LOG_FILE}" 2>&1 &
TF_PID=$!

# 4. EKF
ros2 run robot_localization ekf_node \
    --ros-args \
    -p odom_frame:=odom \
    -p base_link_frame:=base_link \
    -p world_frame:=odom \
    -p imu0:=/imu/data_raw \
    -p use_sim_time:=false \
    -r __node:=ekf_filter_node \
    >> "${LOG_FILE}" 2>&1 &
EKF_PID=$!

# 5. slam_toolbox
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$SLAM_PARAMS" \
    use_sim_time:=false \
    >> "${LOG_FILE}" 2>&1 &
SLAM_PID=$!

# 6. Nav2 (规划+控制,无 AMCL,使用 slam_toolbox 提供的 map→odom TF)
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    autostart:=true \
    params_file:="$NAV2_PARAMS" \
    use_composition:=False \
    use_respawn:=False \
    >> "${LOG_FILE}" 2>&1 &
NAV_PID=$!

# 7. frontier_explorer
ros2 run lidar_slam_nodes frontier_explorer \
    --ros-args \
    --params-file "$FRONTIER_PARAMS" \
    -p use_sim_time:=false \
    -r __node:=frontier_explorer \
    >> "${LOG_FILE}" 2>&1 &
FE_PID=$!

# 8. RViz2
export DISPLAY=:0
for auth in /run/user/$(id -u)/.mutter-Xwaylandauth.* /home/$(whoami)/.Xauthority; do
    if [ -f "$auth" ]; then
        export XAUTHORITY="$auth"
        break
    fi
done
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" \
    --ros-args -p use_sim_time:=false \
    >> "${LOG_FILE}" 2>&1 &
RVIZ_PID=$!

echo "启动节点 PID: lidar=$LIDAR_PID car=$CAR_PID tf=$TF_PID ekf=$EKF_PID slam=$SLAM_PID nav=$NAV_PID fe=$FE_PID rviz=$RVIZ_PID"
echo "按 Ctrl+C 停止所有节点..."

cleanup() {
    echo "正在停止所有节点..."
    kill $LIDAR_PID $CAR_PID $TF_PID $EKF_PID $SLAM_PID $NAV_PID $FE_PID $RVIZ_PID 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
wait
