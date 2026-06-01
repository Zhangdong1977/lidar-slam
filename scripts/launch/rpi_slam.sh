#!/bin/bash
# 树莓派小车 SLAM 建图脚本
# 启动 RPLIDAR C1 + car_base_node (STM32) + slam_toolbox 在线建图
#
# 场景定位: 真实小车建图 (无 Nav2, 无 openTCS)
# 数据流: RPLIDAR C1 → /scan
#         car_base_node (STM32 UART) → /odom (编码器) + /imu/data_raw + /PowerVoltage
#         EKF → odom→base_link TF
#         slam_toolbox → /map + TF map→odom
# 适合: 真实环境地图采集
#
# 硬件要求:
#   RPLIDAR C1: /dev/lidar (460800 baud)
#   STM32:      /dev/ttyAMA0 (115200 baud)
#   权限:       当前用户在 dialout 组
#
# 用法:
#   ./scripts/launch/rpi_slam.sh                            # 默认参数
#   ./scripts/launch/rpi_slam.sh /path/to/custom_slam.yaml  # 自定义 SLAM 参数
#
# 配套: 另开终端保存地图
#   ros2 run nav2_map_server map_saver_cli -f ~/maps/raspberry_map

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# ROS2 环境
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER

source /opt/ros/jazzy/setup.bash
# car_base_node 来自独立的 car_base 工作空间
source /home/pi/ros2_ws/install/setup.bash 2>/dev/null || true
source "${PROJECT_DIR}/install/setup.bash"

# 串口设备检查
LIDAR_PORT="${LIDAR_PORT:-/dev/lidar}"
CHASSIS_PORT="${CHASSIS_PORT:-/dev/ttyAMA0}"

if [ ! -e "$LIDAR_PORT" ]; then
    echo "ERROR: 激光雷达串口不存在: $LIDAR_PORT"
    echo "  请检查 RPLIDAR C1 是否插入"
    exit 1
fi
if [ ! -e "$CHASSIS_PORT" ]; then
    echo "ERROR: 底盘串口不存在: $CHASSIS_PORT"
    echo "  请检查 STM32 UART 是否连接"
    exit 1
fi
if [ ! -r "$LIDAR_PORT" ] || [ ! -w "$LIDAR_PORT" ]; then
    echo "WARN: 串口无读写权限,尝试 sudo chmod..."
    sudo chmod 666 "$LIDAR_PORT" "$CHASSIS_PORT" 2>/dev/null || \
        echo "  请手动执行: sudo chmod 666 $LIDAR_PORT $CHASSIS_PORT"
fi

# 日志输出到 log 目录
LOG_DIR="${PROJECT_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/rpi_slam_$(date +%Y-%m-%d_%H-%M-%S).log"

SLAM_PARAMS="${1:-${PROJECT_DIR}/config/slam_toolbox_real.yaml}"

echo "============================================="
echo "  树莓派小车 SLAM 建图场景"
echo "  Profile:  raspberry (实车)"
echo "  雷达:     $LIDAR_PORT (RPLIDAR C1)"
echo "  底盘:     $CHASSIS_PORT (STM32)"
echo "  SLAM参数: $SLAM_PARAMS"
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

# 2. car_base_node (底盘 + 传感器)
ros2 run car_base car_base_node \
    --ros-args \
    -p odom_frame_id:=odom \
    -p robot_frame_id:=base_link \
    -p gyro_frame_id:=imu_link \
    -p use_sim_time:=false \
    -r __node:=car_base_node \
    >> "${LOG_FILE}" 2>&1 &
CAR_PID=$!

# 3. static TF: base_link -> lidar_link (安装位置 0.15m, 朝向 π)
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0.15 \
    --roll 0 --pitch 0 --yaw 3.14159 \
    --frame-id base_link --child-frame-id lidar_link \
    --ros-args -p use_sim_time:=false \
    >> "${LOG_FILE}" 2>&1 &
TF_PID=$!

# 4. EKF (融合 /odom + /imu)
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

# 5. slam_toolbox online_async
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$SLAM_PARAMS" \
    use_sim_time:=false \
    >> "${LOG_FILE}" 2>&1 &
SLAM_PID=$!

# 6. RViz2
RVIZ_CONFIG="${PROJECT_DIR}/config/slam.rviz"
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

echo "启动节点 PID: lidar=$LIDAR_PID car=$CAR_PID tf=$TF_PID ekf=$EKF_PID slam=$SLAM_PID rviz=$RVIZ_PID"
echo "按 Ctrl+C 停止所有节点..."

# 退出时清理
cleanup() {
    echo "正在停止所有节点..."
    kill $LIDAR_PID $CAR_PID $TF_PID $EKF_PID $SLAM_PID $RVIZ_PID 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
wait
