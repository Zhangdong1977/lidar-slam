#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# WARNING: 命名约束 (2026-06-01 vscode 断连事件后的强制规则)
#
# 本脚本用 `pkill -f "<pattern>"` 按命令行字符串匹配进程, 因此:
#   1) 任何调用方脚本 (尤其 scripts/launch/*.sh) 的 **文件名**
#      不能含本脚本里 pk 使用的清理模式子串, 例如:
#        ros2, gazebo, gz, amcl, nav2, rviz2, controller, tf2, ekf,
#        lifecycle, socat, chassis, ros_gz, robot_state, static_transform,
#        parameter_bridge, joint_state, vehicle_controller, gzserver,
#        gzclient, ros2-daemon, ros2cli.daemon
#   2) 如果新增 pk 模式, 必须同步检查 scripts/launch/*.sh 文件名.
#   3) pk() 仅排除 $$ (本进程) + $PARENT_PID (调用方), 不递归保护
#      祖先进程链. 嵌套调用 / 时序边界 case 见 git log.
#   4) 历史教训: 旧名 gazebo_opentcs_nav.sh 含 "gazebo" 子串, 在
#      vscode 集成终端里跑时, 父脚本被 pk pkill -f "gazebo" 误杀,
#      触发 ptyHost 误判 pty 关闭, vscode client 报 "Connection lost".
#      已重命名为 sim_opentcs_nav.sh, 同理其他启动脚本改 sim_*/rpi_*.
# ─────────────────────────────────────────────────────────────────────

set -e

WORKSPACE=${1:-"lidar-slam"}
echo "=== Cleaning up ROS2, Gazebo, and RViz (workspace: $WORKSPACE) ==="

# 关键: 排除调用本脚本的父进程(以及父进程的命令行), 避免 pkill -f "ros2 launch"
# /workspace 等字符串把本进程误杀
# 记录本 shell 自身 PID 与父进程 PID
PARENT_PID=$(ps -o ppid= -p $$ | tr -d ' ')

# pkill 包装函数: 排除当前 shell 及其父进程
# 用法: pk pkill [-9] -f "<pattern>"
#   - 找到 -f 后的 pattern
#   - 用 pgrep -f 找到匹配 PID, 排除 $$ 和 $PARENT_PID
#   - 用 kill 一个个按 PID 发信号(避免 pkill -f + 显式 PID 同时使用的歧义)
pk() {
    local pattern=""
    local signal="SIGTERM"
    local i=1
    while [ $i -le $# ]; do
        local arg="${!i}"
        case "$arg" in
            -9) signal="SIGKILL" ;;
            -f)
                i=$((i + 1))
                pattern="${!i}"
                ;;
        esac
        i=$((i + 1))
    done
    if [ -z "$pattern" ]; then
        # 没有 -f, 退化为直接调用 pkill(理论上不会走这里, 防御性)
        pkill "$@" 2>/dev/null || true
        return
    fi
    local pid
    while IFS= read -r pid; do
        [ -n "$pid" ] || continue
        if [ "$pid" = "$$" ] || [ "$pid" = "$PARENT_PID" ]; then
            continue
        fi
        kill -"$signal" "$pid" 2>/dev/null || true
    done < <(pgrep -f "$pattern" 2>/dev/null)
}

# ── Phase 1: graceful shutdown (SIGTERM) ──────────────────────────
echo -n "Sending SIGTERM to all ROS/Gazebo processes... "

# ROS2 daemon
pk pkill -f "ros2-daemon" 2>/dev/null || true
pk pkill -f "ros2cli.daemon" 2>/dev/null || true

# Workspace nodes (matched by install path)
pk pkill -f "${WORKSPACE}/install/" 2>/dev/null || true

# ros_gz_bridge (system-installed)
pk pkill -f "ros_gz_bridge" 2>/dev/null || true
pk pkill -f "parameter_bridge" 2>/dev/null || true

# System ROS nodes — use pkill to avoid killall's 15-char name limit
pk pkill -f "static_transform_publisher" 2>/dev/null || true
pk pkill -f "robot_state_publisher" 2>/dev/null || true
pk pkill -f "joint_state_publisher" 2>/dev/null || true
pk pkill -f "controller_manager" 2>/dev/null || true
pk pkill -f "rqt_" 2>/dev/null || true

# ROS2 CLI monitoring tools (tf2_echo, topic hz/echo, node list, service call, etc.)
pk pkill -f "tf2_echo" 2>/dev/null || true
pk pkill -f "tf2_ros" 2>/dev/null || true
pk pkill -f "ros2 topic" 2>/dev/null || true
pk pkill -f "ros2 node" 2>/dev/null || true
pk pkill -f "ros2 service" 2>/dev/null || true
pk pkill -f "ros2 action" 2>/dev/null || true
pk pkill -f "ros2 run" 2>/dev/null || true
pk pkill -f "ros2 launch" 2>/dev/null || true
pk pkill -f "ros2 lifecycle" 2>/dev/null || true
pk pkill -f "ros2 param" 2>/dev/null || true
pk pkill -f "ros2 doctor" 2>/dev/null || true

# robot_localization (EKF)
pk pkill -f "ekf_node" 2>/dev/null || true
pk pkill -f "robot_localization" 2>/dev/null || true

# ackermann_control (vehicle_controller)
pk pkill -f "vehicle_controller" 2>/dev/null || true

# Nav2 nodes (system-installed, not caught by workspace pattern)
pk pkill -f "amcl" 2>/dev/null || true
pk pkill -f "nav2_map_server" 2>/dev/null || true
pk pkill -f "map_saver" 2>/dev/null || true
pk pkill -f "nav2_controller" 2>/dev/null || true
pk pkill -f "nav2_planner" 2>/dev/null || true
pk pkill -f "bt_navigator" 2>/dev/null || true
pk pkill -f "behavior_server" 2>/dev/null || true
pk pkill -f "lifecycle_manager" 2>/dev/null || true
pk pkill -f "waypoint_follower" 2>/dev/null || true
pk pkill -f "nav2_costmap" 2>/dev/null || true
pk pkill -f "recoveries_server" 2>/dev/null || true
pk pkill -f "nav2_smoother" 2>/dev/null || true
pk pkill -f "velocity_smoother" 2>/dev/null || true
pk pkill -f "collision_monitor" 2>/dev/null || true
pk pkill -f "route_server" 2>/dev/null || true
pk pkill -f "nav2_route" 2>/dev/null || true
pk pkill -f "opennav_docking" 2>/dev/null || true

# socat (virtual serial port pair for RS-485 bridge)
pk pkill -f "socat.*chassis" 2>/dev/null || true

# Gazebo classic + modern (gz sim)
pk pkill -f "gzserver" 2>/dev/null || true
pk pkill -f "gzclient" 2>/dev/null || true
pk pkill -f "gazebo" 2>/dev/null || true
pk pkill -f "gz sim" 2>/dev/null || true

# RViz2
pk pkill -f "rviz2" 2>/dev/null || true

echo "done"

# Give processes a moment to exit
sleep 0.8

# ── Phase 2: force kill anything still alive (SIGKILL) ────────────
echo -n "Force-killing survivors... "

pk pkill -9 -f "ros2-daemon" 2>/dev/null || true
pk pkill -9 -f "ros2cli.daemon" 2>/dev/null || true
pk pkill -9 -f "${WORKSPACE}/install/" 2>/dev/null || true
pk pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pk pkill -9 -f "parameter_bridge" 2>/dev/null || true
pk pkill -9 -f "static_transform_publisher" 2>/dev/null || true
pk pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pk pkill -9 -f "joint_state_publisher" 2>/dev/null || true
pk pkill -9 -f "controller_manager" 2>/dev/null || true
pk pkill -9 -f "rqt_" 2>/dev/null || true
pk pkill -9 -f "tf2_echo" 2>/dev/null || true
pk pkill -9 -f "tf2_ros" 2>/dev/null || true
pk pkill -9 -f "ros2 topic" 2>/dev/null || true
pk pkill -9 -f "ros2 node" 2>/dev/null || true
pk pkill -9 -f "ros2 service" 2>/dev/null || true
pk pkill -9 -f "ros2 action" 2>/dev/null || true
pk pkill -9 -f "ros2 run" 2>/dev/null || true
pk pkill -9 -f "ros2 launch" 2>/dev/null || true
pk pkill -9 -f "ros2 lifecycle" 2>/dev/null || true
pk pkill -9 -f "ros2 param" 2>/dev/null || true
pk pkill -9 -f "ros2 doctor" 2>/dev/null || true
pk pkill -9 -f "ekf_node" 2>/dev/null || true
pk pkill -9 -f "robot_localization" 2>/dev/null || true
pk pkill -9 -f "vehicle_controller" 2>/dev/null || true
pk pkill -9 -f "amcl" 2>/dev/null || true
pk pkill -9 -f "nav2_map_server" 2>/dev/null || true
pk pkill -9 -f "map_saver" 2>/dev/null || true
pk pkill -9 -f "nav2_controller" 2>/dev/null || true
pk pkill -9 -f "nav2_planner" 2>/dev/null || true
pk pkill -9 -f "bt_navigator" 2>/dev/null || true
pk pkill -9 -f "behavior_server" 2>/dev/null || true
pk pkill -9 -f "lifecycle_manager" 2>/dev/null || true
pk pkill -9 -f "waypoint_follower" 2>/dev/null || true
pk pkill -9 -f "nav2_costmap" 2>/dev/null || true
pk pkill -9 -f "recoveries_server" 2>/dev/null || true
pk pkill -9 -f "nav2_smoother" 2>/dev/null || true
pk pkill -9 -f "velocity_smoother" 2>/dev/null || true
pk pkill -9 -f "collision_monitor" 2>/dev/null || true
pk pkill -9 -f "route_server" 2>/dev/null || true
pk pkill -9 -f "nav2_route" 2>/dev/null || true
pk pkill -9 -f "opennav_docking" 2>/dev/null || true
pk pkill -9 -f "socat.*chassis" 2>/dev/null || true
pk pkill -9 -f "gzserver" 2>/dev/null || true
pk pkill -9 -f "gzclient" 2>/dev/null || true
pk pkill -9 -f "gazebo" 2>/dev/null || true
pk pkill -9 -f "gz sim" 2>/dev/null || true
pk pkill -9 -f "rviz2" 2>/dev/null || true

echo "done"

# ── Temp files ────────────────────────────────────────────────────
echo -n "Cleaning ROS2 temp files... "
rm -f /tmp/ros2_daemon_* 2>/dev/null || true
rm -rf /tmp/.ros/ 2>/dev/null || true
# RS-485 virtual serial port leftovers
rm -f /tmp/chassis_cmd /tmp/chassis_recv /tmp/*_chassis_cmd /tmp/*_chassis_recv 2>/dev/null || true
echo "done"

# ── Fast-DDS shared memory (file-based, ROS2 Humble+) ─────────────
echo -n "Cleaning Fast-DDS shared memory... "
find /dev/shm -maxdepth 1 -user "$(id -u)" \( -name "fastrtps_*" -o -name "*ros*" \) -delete 2>/dev/null || true
echo "done"

# ── SysV shared memory (current user only) ────────────────────────
if command -v ipcs &>/dev/null && command -v ipcrm &>/dev/null; then
    echo -n "Cleaning orphaned SysV shared memory... "
    ipcs -m 2>/dev/null | awk -v uid="$(id -u)" '$3 == uid {print $2}' \
        | xargs -r -n1 ipcrm -m 2>/dev/null || true
    echo "done"
fi

echo "=== Cleanup complete ==="
