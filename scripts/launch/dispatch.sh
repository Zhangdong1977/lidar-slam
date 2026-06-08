#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# 场景3: 调度集成 (Dispatch Integration with openTCS)
#
# 启动硬件层 + EKF + AMCL定位 + Nav2导航 + openTCS车辆桥接
# 数据流: 传感器 → EKF → AMCL → Nav2 → /cmd_vel
#         opentcs_vehicle_node 上报位姿/电量 → openTCS Kernel
#         openTCS 下发路径 → opentcs_vehicle_node → Nav2 NavigateToPose
#
# 用法:
#   ./scripts/launch/dispatch.sh --discovery-address 192.168.1.10  # 默认: gazebo 仿真 (gazebo_1)
#   ./scripts/launch/dispatch.sh --namespace gazebo_2 --discovery-address 192.168.1.10
#   ./scripts/launch/dispatch.sh --namespace gazebo_2 --spawn-y 2.0 --discovery-address 192.168.1.10
#   ./scripts/launch/dispatch.sh --profile raspberry --namespace c30_1 --discovery-address 192.168.1.10
#   ./scripts/launch/dispatch.sh --profile rs485 --discovery-address 192.168.1.10
#   ./scripts/launch/dispatch.sh --map /path/to/map.yaml --discovery-address 192.168.1.10
#   ./scripts/launch/dispatch.sh --no-rviz --discovery-address 192.168.1.10
#   ./scripts/launch/dispatch.sh --discovery-address 192.168.1.10  # 连接 Sidecar 提供的 Discovery Server
#   ./scripts/launch/dispatch.sh --no-discovery-server          # 使用 DDS multicast 发现
#   ./scripts/launch/dispatch.sh status --namespace gazebo_1 --discovery-address 192.168.1.10
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 默认参数
PROFILE="gazebo"
MAP_FILE="${PROJECT_DIR}/maps/auto_exploration_map.yaml"
DOMAIN_ID=""
NAMESPACE="gazebo_1"
USE_RVIZ="True"
SPAWN_X=""
SPAWN_Y=""
SPAWN_Z="0.24"
INITIAL_POSE_X=""
INITIAL_POSE_Y=""
INITIAL_POSE_YAW="0.0"
SKIP_CLEANUP="False"
START_GAZEBO="True"
USE_DISCOVERY_SERVER="True"
DISCOVERY_SERVER_ADDRESS=""
DISCOVERY_SERVER_PORT="11811"
ACTION="start"
STARTUP_STATUS_DELAY="${DISPATCH_STARTUP_STATUS_DELAY:-15}"
NODE_INFO_TIMEOUT="${DISPATCH_NODE_INFO_TIMEOUT:-3}"
TOPIC_INFO_TIMEOUT="${DISPATCH_TOPIC_INFO_TIMEOUT:-3}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m'

usage() {
    echo "用法: $0 [status] [--profile gazebo|rs485|raspberry] [--namespace NAME] [--spawn-x X] [--spawn-y Y] [--spawn-z Z] [--initial-x X] [--initial-y Y] [--initial-yaw YAW] [--map FILE] [--no-rviz] [--skip-cleanup] [--no-gazebo] [--domain-id ID] [--discovery-address HOST] [--discovery-port PORT] [--no-discovery-server]"
}

# 解析参数
if [ "${1:-}" = "status" ]; then
    ACTION="status"
    shift
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --map)
            MAP_FILE="$2"
            shift 2
            ;;
        --domain-id)
            DOMAIN_ID="$2"
            shift 2
            ;;
        --namespace)
            NAMESPACE="$2"
            shift 2
            ;;
        --spawn-x)
            SPAWN_X="$2"
            shift 2
            ;;
        --spawn-y)
            SPAWN_Y="$2"
            shift 2
            ;;
        --spawn-z)
            SPAWN_Z="$2"
            shift 2
            ;;
        --initial-x)
            INITIAL_POSE_X="$2"
            shift 2
            ;;
        --initial-y)
            INITIAL_POSE_Y="$2"
            shift 2
            ;;
        --initial-yaw)
            INITIAL_POSE_YAW="$2"
            shift 2
            ;;
        --no-rviz)
            USE_RVIZ="False"
            shift
            ;;
        --skip-cleanup)
            SKIP_CLEANUP="True"
            shift
            ;;
        --no-gazebo)
            START_GAZEBO="False"
            shift
            ;;
        --discovery-address)
            DISCOVERY_SERVER_ADDRESS="$2"
            shift 2
            ;;
        --discovery-port)
            DISCOVERY_SERVER_PORT="$2"
            shift 2
            ;;
        --no-discovery-server)
            USE_DISCOVERY_SERVER="False"
            shift
            ;;
        *)
            echo "未知参数: $1"
            usage
            exit 1
            ;;
    esac
done

# ── 环境初始化 ────────────────────────────────────────────────────
# 从 profile YAML 读取 domain_id（默认 42）
DEFAULT_DOMAIN=$(python3 -c "import yaml; print(yaml.safe_load(open('${PROJECT_DIR}/config/profiles/${PROFILE}.yaml')).get('domain_id', 42))" 2>/dev/null || echo 42)
export ROS_DOMAIN_ID="${DOMAIN_ID:-$DEFAULT_DOMAIN}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [ "$PROFILE" = "raspberry" ]; then
    source /opt/ros/jazzy/setup.bash
    source /home/pi/ros2_ws/install/setup.bash 2>/dev/null || true
    unset ROS_LOCALHOST_ONLY
    source "${PROJECT_DIR}/install/setup.bash"
else
    export DISPLAY=:0
    for auth in /run/user/$(id -u)/.mutter-Xwaylandauth.* /home/$(whoami)/.Xauthority; do
        if [ -f "$auth" ]; then
            export XAUTHORITY="$auth"
            break
        fi
    done

    eval "$(conda shell.bash hook)"
    conda activate lidar_slam
    source /opt/ros/jazzy/setup.bash
    unset ROS_LOCALHOST_ONLY
    source "${PROJECT_DIR}/install/setup.bash"
fi

if [ "$USE_DISCOVERY_SERVER" = "True" ]; then
    if [ -n "$DISCOVERY_SERVER_ADDRESS" ]; then
        if [[ "$DISCOVERY_SERVER_ADDRESS" == *":"* || "$DISCOVERY_SERVER_ADDRESS" == *";"* ]]; then
            export ROS_DISCOVERY_SERVER="$DISCOVERY_SERVER_ADDRESS"
        else
            export ROS_DISCOVERY_SERVER="${DISCOVERY_SERVER_ADDRESS}:${DISCOVERY_SERVER_PORT}"
        fi
    elif [ "$ACTION" = "status" ] && [ -z "${ROS_DISCOVERY_SERVER:-}" ]; then
        USE_DISCOVERY_SERVER="False"
        unset ROS_DISCOVERY_SERVER
    elif [ -z "${ROS_DISCOVERY_SERVER:-}" ]; then
        echo "ERROR: Discovery Server 由 sidecar 端提供。请使用 --discovery-address <sidecar_ip>，"
        echo "       或在启动前导出 ROS_DISCOVERY_SERVER=<sidecar_ip>:<port>。"
        echo "       如需退回 multicast 发现，请使用 --no-discovery-server。"
        exit 1
    fi
else
    unset ROS_DISCOVERY_SERVER
fi

# ── Gazebo 多车初始位姿 ───────────────────────────────────────────
# 默认规则: gazebo_1 -> (0, 0), gazebo_2 -> (0, 2), gazebo_N -> (0, 2*(N-1)).
# 可通过 --spawn-x/--spawn-y 显式覆盖。AMCL 初始位姿默认跟随 Gazebo spawn。
if [ "$PROFILE" = "gazebo" ]; then
    if [ -z "$SPAWN_X" ]; then
        SPAWN_X="0"
    fi
    if [ -z "$SPAWN_Y" ]; then
        if [[ "$NAMESPACE" =~ ^gazebo_([0-9]+)$ ]]; then
            vehicle_index=$((10#${BASH_REMATCH[1]}))
            SPAWN_Y="$(( (vehicle_index - 1) * 2 ))"
        else
            SPAWN_Y="0"
        fi
    fi
fi

as_float_literal() {
    if [[ "$1" =~ ^-?[0-9]+$ ]]; then
        echo "$1.0"
    else
        echo "$1"
    fi
}

if [ -z "$INITIAL_POSE_X" ]; then
    INITIAL_POSE_X="${SPAWN_X:-0.0}"
fi
if [ -z "$INITIAL_POSE_Y" ]; then
    INITIAL_POSE_Y="${SPAWN_Y:-0.0}"
fi
INITIAL_POSE_X="$(as_float_literal "$INITIAL_POSE_X")"
INITIAL_POSE_Y="$(as_float_literal "$INITIAL_POSE_Y")"
INITIAL_POSE_YAW="$(as_float_literal "$INITIAL_POSE_YAW")"

ns_path() {
    local node="$1"
    if [ -n "$NAMESPACE" ]; then
        echo "/${NAMESPACE}/${node}"
    else
        echo "/${node}"
    fi
}

topic_path() {
    local topic="$1"
    if [[ "$topic" == /* ]]; then
        echo "$topic"
    elif [ -n "$NAMESPACE" ]; then
        echo "/${NAMESPACE}/${topic}"
    else
        echo "/${topic}"
    fi
}

run_ros2_quiet() {
    ROS_DISABLE_DAEMON=1 timeout "$@"
}

node_list_snapshot() {
    run_ros2_quiet 5 ros2 node list 2>/dev/null || true
}

topic_info_snapshot() {
    local topic="$1"
    run_ros2_quiet 4 ros2 topic info "$topic" 2>/dev/null || true
}

topic_list_snapshot() {
    run_ros2_quiet "$TOPIC_INFO_TIMEOUT" ros2 topic list --include-hidden-topics -t 2>/dev/null || true
}

topic_info_verbose_snapshot() {
    local topic="$1"
    run_ros2_quiet "$TOPIC_INFO_TIMEOUT" ros2 topic info -v "$topic" 2>/dev/null || true
}

node_info_snapshot() {
    local node="$1"
    run_ros2_quiet "$NODE_INFO_TIMEOUT" ros2 node info --include-hidden "$node" 2>/dev/null || true
}

print_status_line() {
    local state="$1"
    local label="$2"
    local detail="${3:-}"
    case "$state" in
        OK) printf "  ${GREEN}[OK]${NC}      %-30s %s\n" "$label" "$detail" ;;
        WARN) printf "  ${YELLOW}[WAIT]${NC}    %-30s %s\n" "$label" "$detail" ;;
        ERROR) printf "  ${RED}[MISS]${NC}    %-30s %s\n" "$label" "$detail" ;;
        INFO) printf "  ${BLUE}[INFO]${NC}    %-30s %s\n" "$label" "$detail" ;;
        *) printf "  [%s] %-30s %s\n" "$state" "$label" "$detail" ;;
    esac
}

node_present_exact() {
    local nodes="$1"
    local node="$2"
    echo "$nodes" | grep -Fxq "$node"
}

node_present_prefix() {
    local nodes="$1"
    local prefix="$2"
    awk -v prefix="$prefix" 'index($0, prefix) == 1 { found = 1 } END { exit found ? 0 : 1 }' <<< "$nodes"
}

print_expected_node() {
    local nodes="$1"
    local label="$2"
    local node="$3"
    local mode="${4:-exact}"
    if [ "$mode" = "prefix" ]; then
        if node_present_prefix "$nodes" "$node"; then
            print_status_line OK "$label" "$node*"
        else
            print_status_line ERROR "$label" "$node*"
        fi
    elif node_present_exact "$nodes" "$node"; then
        print_status_line OK "$label" "$node"
    else
        print_status_line ERROR "$label" "$node"
    fi
}

print_process_status() {
    local label="$1"
    local pattern="$2"
    if pgrep -f "$pattern" >/dev/null 2>&1; then
        print_status_line OK "$label" "process: $pattern"
    else
        print_status_line ERROR "$label" "process: $pattern"
    fi
}

print_topic_status() {
    local label="$1"
    local topic
    topic="$(topic_path "$2")"
    local info pubs subs
    info="$(topic_info_snapshot "$topic")"
    pubs="$(echo "$info" | awk -F': ' '/Publisher count|Publication count/ {print $2; exit}')"
    subs="$(echo "$info" | awk -F': ' '/Subscription count/ {print $2; exit}')"
    pubs="${pubs:-0}"
    subs="${subs:-0}"
    if [ "$pubs" -gt 0 ] 2>/dev/null; then
        print_status_line OK "$label" "$topic pub=$pubs sub=$subs"
    else
        print_status_line ERROR "$label" "$topic pub=$pubs sub=$subs"
    fi
}

print_node_topic_details() {
    local nodes="$1"
    local ns_prefix="/${NAMESPACE}/"
    local filtered_nodes node info

    if [ -n "$NAMESPACE" ]; then
        filtered_nodes="$(awk -v prefix="$ns_prefix" 'index($0, prefix) == 1' <<< "$nodes")"
    else
        filtered_nodes="$nodes"
    fi

    echo ""
    echo "--- 节点关联话题 (${NAMESPACE:-global}) ---"
    if [ -z "$(echo "$filtered_nodes" | sed '/^$/d')" ]; then
        print_status_line WARN "node topics" "未发现当前 namespace 下的节点"
        return 0
    fi

    while IFS= read -r node; do
        [ -n "$node" ] || continue
        info="$(node_info_snapshot "$node")"
        echo "  $node"
        if [ -z "$info" ]; then
            echo "    ros2 node info 无输出"
            continue
        fi
        awk '
            /^[[:space:]]*Publishers:/ { section = "pub"; next }
            /^[[:space:]]*Subscribers:/ { section = "sub"; next }
            /^[[:space:]]*[[:alpha:]][[:alpha:] ]*:/ { section = ""; next }
            section == "pub" && /^[[:space:]]+\// {
                line = $0
                sub(/^[[:space:]]+/, "", line)
                pubs[++pub_count] = line
                next
            }
            section == "sub" && /^[[:space:]]+\// {
                line = $0
                sub(/^[[:space:]]+/, "", line)
                subs[++sub_count] = line
                next
            }
            END {
                print "    Publishers:"
                if (pub_count == 0) {
                    print "      (none)"
                } else {
                    for (i = 1; i <= pub_count; i++) print "      " pubs[i]
                }
                print "    Subscribers:"
                if (sub_count == 0) {
                    print "      (none)"
                } else {
                    for (i = 1; i <= sub_count; i++) print "      " subs[i]
                }
            }
        ' <<< "$info"
    done <<< "$filtered_nodes"
}

filter_relevant_topics() {
    local topics="$1"
    local ns_prefix="/${NAMESPACE}/"

    if [ -n "$NAMESPACE" ]; then
        awk -v prefix="$ns_prefix" '
            index($0, prefix) == 1 ||
            $1 == "/tf" ||
            $1 == "/tf_static" ||
            $1 == "/parameter_events" ||
            $1 == "/rosout"
        ' <<< "$topics"
    else
        echo "$topics"
    fi
}

print_discovered_topics() {
    local topics="$1"
    local relevant_topics
    relevant_topics="$(filter_relevant_topics "$topics")"

    echo ""
    echo "--- 已发现话题 (${NAMESPACE:-global}) ---"
    if [ -z "$(echo "$relevant_topics" | sed '/^$/d')" ]; then
        print_status_line WARN "topics" "未发现当前 namespace 下的话题"
        return 0
    fi

    echo "$relevant_topics" | sed 's/^/  /'
}

print_topic_endpoint_details() {
    local topics="$1"
    local relevant_topics topic info
    relevant_topics="$(filter_relevant_topics "$topics")"

    echo ""
    echo "--- 话题端点详情 (${NAMESPACE:-global}) ---"
    if [ -z "$(echo "$relevant_topics" | sed '/^$/d')" ]; then
        print_status_line WARN "topic endpoints" "未发现可查询的话题"
        return 0
    fi

    while IFS= read -r topic_line; do
        [ -n "$topic_line" ] || continue
        topic="${topic_line%% *}"
        [ -n "$topic" ] || continue
        info="$(topic_info_verbose_snapshot "$topic")"
        echo "  $topic_line"
        if [ -z "$info" ]; then
            echo "    ros2 topic info 无输出"
            continue
        fi
        awk '
            /^Publisher count:/ {
                print "    " $0
                next
            }
            /^Subscription count:/ {
                print "    " $0
                next
            }
            /^Node name:/ {
                name = $0
                sub(/^Node name:[[:space:]]*/, "", name)
                next
            }
            /^Node namespace:/ {
                ns = $0
                sub(/^Node namespace:[[:space:]]*/, "", ns)
                next
            }
            /^Endpoint type:/ {
                endpoint = $0
                sub(/^Endpoint type:[[:space:]]*/, "", endpoint)
                if (name != "") {
                    if (ns == "/") {
                        print "    " endpoint " /" name
                    } else {
                        print "    " endpoint " " ns "/" name
                    }
                }
                name = ""
                ns = ""
                next
            }
        ' <<< "$info"
    done <<< "$relevant_topics"
}

print_runtime_status() {
    local title="${1:-运行状态}"
    local nodes node_count ns_prefix topics
    nodes="$(node_list_snapshot)"
    topics="$(topic_list_snapshot)"
    node_count="$(echo "$nodes" | sed '/^$/d' | wc -l | tr -d ' ')"
    ns_prefix="/${NAMESPACE}/"

    echo ""
    echo "─────────────────────────────────────────────"
    echo "  $title $(date '+%H:%M:%S')"
    echo "─────────────────────────────────────────────"
    if [ "$node_count" -eq 0 ]; then
        print_status_line ERROR "ROS graph" "未发现任何节点；优先检查 DDS discovery"
    else
        print_status_line OK "ROS graph" "发现 ${node_count} 个节点"
    fi

    echo ""
    echo "--- 硬件 / 仿真 ---"
    if [ "$PROFILE" = "gazebo" ]; then
        print_process_status "Gazebo" "gz sim.*factory.sdf"
        print_expected_node "$nodes" "ros_gz_bridge" "$(ns_path ros_gz_bridge)"
        print_expected_node "$nodes" "robot_state_publisher" "$(ns_path robot_state_publisher)"
        print_expected_node "$nodes" "static_lidar_tf" "$(ns_path static_transform_publisher)" prefix
        print_expected_node "$nodes" "wait_for_scan" "$(ns_path wait_scan)"
        print_expected_node "$nodes" "wait_for_cm" "$(ns_path wait_for_cm)"
        print_expected_node "$nodes" "wait_for_joints" "$(ns_path wait_for_joints)"
        print_expected_node "$nodes" "vehicle_controller" "$(ns_path vehicle_controller)"
        print_expected_node "$nodes" "rs485_receiver" "$(ns_path rs485_chassis_receiver)"
    else
        print_expected_node "$nodes" "wait_for_scan" "$(ns_path wait_scan)"
    fi
    print_expected_node "$nodes" "node_watchdog" "$(ns_path node_watchdog)"

    echo ""
    echo "--- 感知 / 定位 ---"
    print_expected_node "$nodes" "ekf_filter_node" "$(ns_path ekf_filter_node)"
    print_expected_node "$nodes" "wait_ekf_tf" "$(ns_path wait_ekf_tf)"
    print_expected_node "$nodes" "map_server" "$(ns_path map_server)"
    print_expected_node "$nodes" "amcl" "$(ns_path amcl)"
    print_expected_node "$nodes" "lifecycle_localization" "$(ns_path lifecycle_manager_localization)"
    print_expected_node "$nodes" "wait_localization" "$(ns_path wait_localization_ready)"

    echo ""
    echo "--- Nav2 ---"
    for node in \
        controller_server smoother_server planner_server route_server \
        behavior_server velocity_smoother collision_monitor bt_navigator \
        waypoint_follower docking_server lifecycle_manager_navigation; do
        print_expected_node "$nodes" "$node" "$(ns_path "$node")"
    done

    echo ""
    echo "--- 调度 / 应用 ---"
    if [ "$PROFILE" != "raspberry" ]; then
        print_expected_node "$nodes" "cmd_vel_bridge" "$(ns_path cmd_vel_bridge)"
    fi
    print_expected_node "$nodes" "opentcs_vehicle" "$(ns_path opentcs_vehicle_node)"
    print_expected_node "$nodes" "lifecycle_custom" "$(ns_path lifecycle_starter_custom)"
    print_expected_node "$nodes" "wait_route" "$(ns_path wait_route)"
    print_expected_node "$nodes" "route_graph_loader" "$(ns_path route_graph_loader)"
    print_expected_node "$nodes" "material_action" "$(ns_path material_action_server)"

    echo ""
    echo "--- 关键话题 ---"
    if [ "$PROFILE" = "gazebo" ]; then
        print_topic_status "robot_description" "robot_description"
    fi
    print_topic_status "scan" "scan"
    print_topic_status "odom" "odom"
    print_topic_status "imu" "imu"
    print_topic_status "map" "map"
    print_topic_status "cmd_vel" "cmd_vel"

    print_discovered_topics "$topics"
    print_topic_endpoint_details "$topics"

    if [ "$node_count" -gt 0 ]; then
        echo ""
        echo "--- 已发现节点 (${NAMESPACE:-global}) ---"
        if [ -n "$NAMESPACE" ]; then
            awk -v prefix="$ns_prefix" 'index($0, prefix) == 1' <<< "$nodes" | sed 's/^/  /' || true
        else
            echo "$nodes" | sed 's/^/  /'
        fi
    fi

    if [ "$node_count" -gt 0 ]; then
        print_node_topic_details "$nodes"
    fi
}

discovery_host_port_rows() {
    echo "${ROS_DISCOVERY_SERVER:-}" | tr ';' '\n' | sed '/^$/d'
}

check_discovery_server() {
    if [ "$USE_DISCOVERY_SERVER" != "True" ]; then
        echo ""
        echo "--- DDS Discovery 检查 ---"
        print_status_line INFO "模式" "multicast，跳过 Discovery Server 探针"
        return 0
    fi

    echo ""
    echo "--- DDS Discovery Server 检查 ---"
    print_status_line INFO "配置" "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER}"
    print_status_line INFO "Domain" "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, RMW=${RMW_IMPLEMENTATION}"

    local endpoint host port route_found=0
    while IFS= read -r endpoint; do
        [ -n "$endpoint" ] || continue
        host="${endpoint%:*}"
        port="${endpoint##*:}"
        if [ "$host" = "$port" ]; then
            port="11811"
        fi
        if [ -z "$host" ]; then
            continue
        fi

        if command -v ip >/dev/null 2>&1; then
            local route
            route="$(ip route get "$host" 2>/dev/null | head -1 || true)"
            if [ -n "$route" ]; then
                route_found=1
                print_status_line OK "路由" "$route"
            else
                print_status_line ERROR "路由" "无法路由到 $host"
            fi
        fi

        if command -v ping >/dev/null 2>&1; then
            if timeout 3 ping -c 1 -W 1 "$host" >/dev/null 2>&1; then
                print_status_line OK "主机连通" "$host"
            else
                print_status_line WARN "主机连通" "$host ping 无响应；可能被防火墙禁 ICMP"
            fi
        fi

        if command -v nc >/dev/null 2>&1; then
            if timeout 3 nc -uvz -w 1 "$host" "$port" >/dev/null 2>&1; then
                print_status_line INFO "UDP探针" "$host:$port 可发送 UDP 探测包"
            else
                print_status_line WARN "UDP探针" "$host:$port 未得到 nc 成功结果"
            fi
        fi
    done < <(discovery_host_port_rows)

    if [ "$route_found" -eq 0 ]; then
        print_status_line WARN "路由" "未能解析 Discovery Server endpoint"
    fi

    local probe_topic probe_payload echo_log echo_pid pub_status
    probe_topic="/dispatch_discovery_probe_${NAMESPACE:-global}_$$"
    probe_payload="discovery_probe_$$"
    echo_log="$(mktemp /tmp/dispatch_discovery_echo.XXXXXX)"

    ROS_DISABLE_DAEMON=1 timeout 8 ros2 topic echo "$probe_topic" std_msgs/msg/String --once \
        >"$echo_log" 2>&1 &
    echo_pid=$!
    sleep 1.0

    set +e
    ROS_DISABLE_DAEMON=1 timeout 4 ros2 topic pub --once "$probe_topic" std_msgs/msg/String \
        "{data: '${probe_payload}'}" >/dev/null 2>&1
    pub_status=$?
    wait "$echo_pid"
    local echo_status=$?
    set -e

    if [ "$pub_status" -eq 0 ] && [ "$echo_status" -eq 0 ] && grep -q "$probe_payload" "$echo_log"; then
        print_status_line OK "ROS发现探针" "两个临时 ROS 2 节点可通过当前 DDS 配置互相发现"
        rm -f "$echo_log"
        return 0
    fi

    print_status_line ERROR "ROS发现探针" "临时 pub/echo 未互相发现"
    echo "  诊断提示:"
    echo "    - 确认 sidecar 上已启动: fastdds discovery -i 0 -l <sidecar_ip> -p ${DISCOVERY_SERVER_PORT}"
    echo "    - 确认 UDP ${DISCOVERY_SERVER_PORT} 未被防火墙拦截"
    echo "    - 确认 ROS_DISCOVERY_SERVER 中的 server-id 位置与 fastdds discovery -i 一致"
    echo "    - 如需退回 multicast，请使用 --no-discovery-server"
    echo "  pub 状态:  $pub_status"
    echo "  echo 状态: $echo_status"
    sed 's/^/  echo: /' "$echo_log" | tail -20
    rm -f "$echo_log"
    return 1
}

if [ "$ACTION" = "status" ]; then
    print_runtime_status "当前状态"
    exit 0
fi

# ── 地图检查 ──────────────────────────────────────────────────────
if [ ! -f "$MAP_FILE" ]; then
    echo "ERROR: 地图文件不存在: $MAP_FILE"
    echo "用法: $0 [--map FILE]"
    exit 1
fi

# ── 日志 ──────────────────────────────────────────────────────────
LOG_DIR="${PROJECT_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/dispatch_${PROFILE}_$(date +%Y-%m-%d_%H-%M-%S).log"

echo "============================================="
echo "  调度集成场景"
echo "  Profile:   $PROFILE"
echo "  Namespace: ${NAMESPACE:-'(none)'}"
echo "  Spawn:     x=${SPAWN_X:-'(default)'} y=${SPAWN_Y:-'(default)'} z=${SPAWN_Z:-'(default)'}"
echo "  AMCL init: x=${INITIAL_POSE_X} y=${INITIAL_POSE_Y} yaw=${INITIAL_POSE_YAW}"
echo "  RViz:      $USE_RVIZ"
echo "  Domain:    $ROS_DOMAIN_ID"
if [ "$USE_DISCOVERY_SERVER" = "True" ]; then
    echo "  DDS发现:   Discovery Server client ${ROS_DISCOVERY_SERVER} (sidecar 提供)"
else
    echo "  DDS发现:   multicast"
fi
echo "  地图:      $MAP_FILE"
echo "  日志:      ${LOG_FILE}"
echo "============================================="

# ── Discovery Server 检查 ─────────────────────────────────────────
if ! check_discovery_server; then
    echo ""
    echo "ERROR: Discovery Server 检查失败，停止启动调度场景。"
    exit 2
fi

# ── 清理残留 ──────────────────────────────────────────────────────
if [ "$SKIP_CLEANUP" = "True" ]; then
    echo "跳过 ROS2/Gazebo 清理 (--skip-cleanup)"
else
    bash "${PROJECT_DIR}/scripts/tools/cleanup_ros2.sh"
fi

# ── 启动 ──────────────────────────────────────────────────────────
shutdown_launch() {
    local code=$?
    if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo ""
        echo "正在停止 dispatch launch (pid=$LAUNCH_PID)..."
        kill -TERM "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
    print_runtime_status "最终状态"
    exit "$code"
}

trap shutdown_launch INT TERM

ros2 launch "${PROJECT_DIR}/launch/nav_main.launch.py" \
    hardware_profile:="${PROFILE}" \
    use_respawn:=True \
    use_rviz:="${USE_RVIZ}" \
    vehicle_name:="${NAMESPACE}" \
    namespace:="${NAMESPACE}" \
    map_file:="$MAP_FILE" \
    spawn_x:="$SPAWN_X" \
    spawn_y:="$SPAWN_Y" \
    spawn_z:="$SPAWN_Z" \
    initial_pose_x:="$INITIAL_POSE_X" \
    initial_pose_y:="$INITIAL_POSE_Y" \
    initial_pose_yaw:="$INITIAL_POSE_YAW" \
    start_gazebo:="$START_GAZEBO" \
    < /dev/null \
    >> "${LOG_FILE}" 2>&1 &

LAUNCH_PID=$!
echo ""
echo "dispatch launch 已启动: pid=${LAUNCH_PID}"
echo "启动状态将在 ${STARTUP_STATUS_DELAY}s 后打印一次 (可用 DISPATCH_STARTUP_STATUS_DELAY 覆盖)"
echo "可手动查询状态: $0 status --namespace ${NAMESPACE:-global}"
echo "日志: ${LOG_FILE}"

sleep "$STARTUP_STATUS_DELAY"
if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    print_runtime_status "启动稳定状态"
fi

set +e
wait "$LAUNCH_PID"
LAUNCH_EXIT=$?
set -e
print_runtime_status "最终状态"
exit "$LAUNCH_EXIT"
