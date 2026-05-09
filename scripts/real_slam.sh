#!/bin/bash
# 启动 RPLIDAR S2 在线 SLAM 建图
#
# 用法:
#   ./scripts/real_slam.sh all      # 建图 + RViz (默认)
#   ./scripts/real_slam.sh mapping  # 仅建图（无RViz，省CPU）
#   ./scripts/real_slam.sh rviz     # 仅RViz（需先启动mapping）
#
# 注意: mapping 和 rviz 必须在同一台机器或网络内

set -e

# 项目根目录: 优先使用环境变量，否则从脚本位置自动探测
: "${LIDAR_SLAM_ROOT:=$(cd "$(dirname "$0")/.." && pwd)}"
export LIDAR_SLAM_ROOT
echo "[INFO] LIDAR_SLAM_ROOT=$LIDAR_SLAM_ROOT"

# ROS2 环境
# 局域网远程查看: 两边设置相同的 ROS_DOMAIN_ID
: "${ROS_DOMAIN_ID:=42}"
export ROS_DOMAIN_ID
echo "[INFO] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

source /opt/ros/jazzy/setup.bash
source "$LIDAR_SLAM_ROOT/third-party/rplidar_ws/install/setup.bash"

LAUNCH_FILE="$LIDAR_SLAM_ROOT/launch/real_slam.launch.py"
RVIZ_CONFIG="$LIDAR_SLAM_ROOT/config/slam.rviz"

MODE="${1:-all}"

# 仅 all 和 mapping 需要串口
if [ "$MODE" = "all" ] || [ "$MODE" = "mapping" ]; then
    SERIAL_PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
    if [ -z "$SERIAL_PORT" ]; then
        echo "[ERROR] 未找到串口设备 /dev/ttyUSB*"
        exit 1
    fi
    echo "[INFO] 使用串口: $SERIAL_PORT"

    if [ ! -w "$SERIAL_PORT" ]; then
        echo "[WARN] $SERIAL_PORT 无写入权限，尝试修复..."
        sudo chmod 666 "$SERIAL_PORT"
    fi
fi

case "$MODE" in
    all)
        echo "[INFO] 启动模式: 建图 + RViz"
        export DISPLAY=:0
        for auth in /run/user/$(id -u)/.mutter-Xwaylandauth.* /home/$(whoami)/.Xauthority; do
            if [ -f "$auth" ]; then
                export XAUTHORITY="$auth"
                break
            fi
        done
        exec ros2 launch "$LAUNCH_FILE" serial_port:="$SERIAL_PORT" rviz:=true
        ;;

    mapping)
        echo "[INFO] 启动模式: 仅建图（无RViz）"
        exec ros2 launch "$LAUNCH_FILE" serial_port:="$SERIAL_PORT" rviz:=false
        ;;

    rviz)
        echo "[INFO] 启动模式: 仅RViz"
        export DISPLAY=:0
        for auth in /run/user/$(id -u)/.mutter-Xwaylandauth.* /home/$(whoami)/.Xauthority; do
            if [ -f "$auth" ]; then
                export XAUTHORITY="$auth"
                break
            fi
        done
        exec rviz2 -d "$RVIZ_CONFIG"
        ;;

    *)
        echo "用法: $0 {all|mapping|rviz}"
        echo ""
        echo "  all     - 建图 + RViz (默认)"
        echo "  mapping - 仅建图，无RViz，省CPU"
        echo "  rviz    - 仅启动RViz，需先运行 mapping"
        exit 1
        ;;
esac
