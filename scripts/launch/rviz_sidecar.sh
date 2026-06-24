#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# 一键在 sidecar (10.0.0.187) 部署并启动 RViz，查看车端 (10.0.0.60) Gazebo 仿真。
#
# 在车端本机运行：通过 sshpass ssh/scp 把运行时文件同步到 sidecar，
# 必要时安装 ros-jazzy-rviz2，再远程拉起 _sidecar_rviz.sh。
# 画面经 sidecar VNC（默认 :1，RFB 5901）显示。
#
# 用法:
#   ./scripts/launch/rviz_sidecar.sh                 # deploy + 前台 start
#   ./scripts/launch/rviz_sidecar.sh deploy          # 仅同步文件 + 装 rviz2
#   ./scripts/launch/rviz_sidecar.sh start           # 前台远程启动
#   ./scripts/launch/rviz_sidecar.sh start --bg      # 后台启动 (nohup)，立即返回
#   ./scripts/launch/rviz_sidecar.sh stop            # 远程停止 rviz2 + bridge
#   ./scripts/launch/rviz_sidecar.sh status          # 远程 ros2 topic list 验证
#   ./scripts/launch/rviz_sidecar.sh --namespaces gazebo_1
#
# 选项: [--namespaces ns1,ns2] [--domain-id 42] [--display :1]
#       [--sidecar pi@10.0.0.187] [--password 12345678] [--bg] [--no-deploy]
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

SIDECAR="pi@10.0.0.187"
PASSWORD="12345678"
REMOTE_DIR_NAME="lidar-slam-rviz"     # 远端目录名（放在 ~ 下）
DOMAIN_ID="42"
NAMESPACES="gazebo_1,gazebo_2"
DISPLAY_VAL=":1"
BG="False"
SKIP_DEPLOY="False"
COMMAND="run"

# 先剥离子命令（允许放在最前）
if [[ $# -gt 0 ]]; then
    case "$1" in
        run|deploy|start|stop|status) COMMAND="$1"; shift ;;
        --*) ;;  # 无子命令，走默认 run
        *)
            echo "未知子命令或参数: $1"
            echo "用法: $0 [run|deploy|start|stop|status] [选项]"
            exit 1
            ;;
    esac
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        --namespaces) NAMESPACES="$2"; shift 2 ;;
        --domain-id) DOMAIN_ID="$2"; shift 2 ;;
        --display) DISPLAY_VAL="$2"; shift 2 ;;
        --sidecar) SIDECAR="$2"; shift 2 ;;
        --password) PASSWORD="$2"; shift 2 ;;
        --bg) BG="True"; shift ;;
        --no-deploy) SKIP_DEPLOY="True"; shift ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [run|deploy|start|stop|status] [--namespaces ns] [--domain-id ID] [--display :N] [--sidecar HOST] [--password PW] [--bg] [--no-deploy]"
            exit 1
            ;;
    esac
done

command -v sshpass >/dev/null 2>&1 || { echo "ERROR: 缺少 sshpass，请 sudo apt-get install -y sshpass"; exit 1; }

SSH_OPTS=(-o StrictHostKeyChecking=no -o ConnectTimeout=15)
SSHC=(sshpass -p "$PASSWORD" ssh "${SSH_OPTS[@]}" "$SIDECAR")
SCPC=(sshpass -p "$PASSWORD" scp "${SSH_OPTS[@]}")

# 远端绝对目录用 $HOME（在本脚本双引号内写 \$HOME，传字面到远端由远端 shell 展开）
REMOTE_HOME_DIR="\$HOME/${REMOTE_DIR_NAME}"

run_deploy() {
    echo ">>> [deploy] 同步运行时文件到 ${SIDECAR}:~/${REMOTE_DIR_NAME}/"
    "${SSHC[@]}" "mkdir -p ~/${REMOTE_DIR_NAME}"
    "${SCPC[@]}" \
        "${PROJECT_DIR}/src/lidar_slam_nodes/lidar_slam_nodes/rviz_tf_bridge.py" \
        "${PROJECT_DIR}/config/cyclonedds.sidecar.sim.xml" \
        "${PROJECT_DIR}/config/nav_multi.rviz" \
        "${SCRIPT_DIR}/_sidecar_rviz.sh" \
        "${SIDECAR}:~/${REMOTE_DIR_NAME}/"
    "${SSHC[@]}" "chmod +x ${REMOTE_HOME_DIR}/_sidecar_rviz.sh"

    echo ">>> [deploy] 检查/安装 ros-jazzy-rviz2"
    "${SSHC[@]}" "if command -v rviz2 >/dev/null 2>&1; then
        echo \"rviz2 已安装: \$(which rviz2)\";
    else
        echo 'rviz2 缺失，安装 ros-jazzy-rviz2 ...';
        echo '${PASSWORD}' | sudo -S apt-get install -y ros-jazzy-rviz2 && echo 'rviz2 安装完成';
    fi"
}

do_start() {
    # 本地展开 NAMESPACES/DOMAIN_ID/DISPLAY_VAL，\$HOME 传字面由远端展开
    local remote_cmd="bash ${REMOTE_HOME_DIR}/_sidecar_rviz.sh --namespaces '${NAMESPACES}' --domain-id ${DOMAIN_ID} --display ${DISPLAY_VAL}"
    if [ "$BG" = "True" ]; then
        echo ">>> [start] 后台启动 (日志: ${SIDECAR}:~/${REMOTE_DIR_NAME}/rviz.log)"
        "${SSHC[@]}" "nohup ${remote_cmd} > ~/${REMOTE_DIR_NAME}/rviz.log 2>&1 < /dev/null & echo \"已后台启动，PID: \$!\""
        echo ">>> 请在 VNC 查看 ${SIDECAR%:*} 的桌面 ${DISPLAY_VAL}（RFB 端口 5901）"
    else
        echo ">>> [start] 前台启动（在 VNC 查看；此处 Ctrl-C 退出 rviz 并清理 bridge）"
        sshpass -p "$PASSWORD" ssh -t "${SSH_OPTS[@]}" "$SIDECAR" "$remote_cmd"
    fi
}

do_stop() {
    echo ">>> [stop] 停止 sidecar 上的 rviz2 / rviz_tf_bridge"
    "${SSHC[@]}" "pkill -f 'rviz2' 2>/dev/null || true; pkill -f 'rviz_tf_bridge.py' 2>/dev/null || true; echo done"
}

do_status() {
    echo ">>> [status] 远程话题发现 (DOMAIN_ID=${DOMAIN_ID}, peer=10.0.0.60)"
    "${SSHC[@]}" "source /opt/ros/jazzy/setup.bash 2>/dev/null; \
        export ROS_DOMAIN_ID=${DOMAIN_ID}; \
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
        export CYCLONEDDS_URI=file://${REMOTE_HOME_DIR}/cyclonedds.sidecar.sim.xml; \
        ros2 topic list 2>/dev/null | head -40 || true"
    echo ">>> 若看不到 /gazebo_1/... 话题，请确认车端 dispatch.sh 已用 --discovery-address ${SIDECAR#*@} 启动"
}

case "$COMMAND" in
    deploy) run_deploy ;;
    start)  do_start ;;
    stop)   do_stop ;;
    status) do_status ;;
    run)
        if [ "$SKIP_DEPLOY" != "True" ]; then run_deploy; fi
        do_start
        ;;
esac
