#!/bin/bash
# Ackermann 仿真诊断脚本 - 检查完整数据链路
# 用法: 启动仿真后，在另一个终端运行 ./scripts/diagnose_ackermann.sh

eval "$(conda shell.bash hook)"
conda activate lidar_slam
source /opt/ros/jazzy/setup.bash
LIDAR_SLAM_ROOT="${LIDAR_SLAM_ROOT:-/home/hello/lidar-slam}"
source "${LIDAR_SLAM_ROOT}/install/setup.bash"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "  Ackermann 仿真诊断"
echo "=========================================="
echo ""

# 1. 检查关键节点是否在运行
echo "--- 1. 检查关键节点 ---"
for node in vehicle_controller ackermann_teleop robot_state_publisher controller_manager; do
    if ros2 node list 2>/dev/null | grep -q "$node"; then
        echo -e "  ${GREEN}[OK]${NC} 节点 $node 在运行"
    else
        echo -e "  ${RED}[MISSING]${NC} 节点 $node 未找到"
    fi
done
echo ""

# 2. 检查控制器状态
echo "--- 2. 检查控制器状态 ---"
CONTROLLERS=$(ros2 control list_controllers 2>/dev/null)
if [ -z "$CONTROLLERS" ]; then
    echo -e "  ${RED}[ERROR]${NC} 无法获取控制器列表 (controller_manager 可能未运行)"
else
    echo "$CONTROLLERS" | while IFS= read -r line; do
        if echo "$line" | grep -q "active"; then
            echo -e "  ${GREEN}[OK]${NC} $line"
        else
            echo -e "  ${RED}[ISSUE]${NC} $line"
        fi
    done
fi
echo ""

# 3. 检查话题发布频率
echo "--- 3. 检查话题 (等待 3 秒采样) ---"
for topic in /steering_angle /velocity /forward_position_controller/commands /forward_velocity_controller/commands /joint_states; do
    FREQ=$(timeout 3 ros2 topic hz "$topic" 2>&1 | tail -1 | grep -oP '[\d.]+')
    if [ -n "$FREQ" ]; then
        echo -e "  ${GREEN}[OK]${NC} $topic → ${FREQ} Hz"
    else
        SUBS=$(ros2 topic info "$topic" 2>/dev/null | grep "Subscription count" | grep -oP '\d+')
        PUBS=$(ros2 topic info "$topic" 2>/dev/null | grep "Publication count" | grep -oP '\d+')
        if [ "$PUBS" = "0" ] 2>/dev/null; then
            echo -e "  ${RED}[NO PUB]${NC} $topic → 没有发布者!"
        elif [ "$SUBS" = "0" ] 2>/dev/null; then
            echo -e "  ${YELLOW}[NO SUB]${NC} $topic → 没有订阅者!"
        else
            echo -e "  ${YELLOW}[NO DATA]${NC} $topic → 有发布/订阅但无数据"
        fi
    fi
done
echo ""

# 4. 检查关节状态 (采样一次)
echo "--- 4. 关节状态 (采样 1 条消息) ---"
JOINT_MSG=$(timeout 3 ros2 topic echo /joint_states --once 2>/dev/null)
if [ -n "$JOINT_MSG" ]; then
    echo "$JOINT_MSG" | grep -A1 "name:" | head -20
    echo ""
    echo "  位置:"
    echo "$JOINT_MSG" | grep -A1 "position:" | head -20
    echo ""
    echo "  速度:"
    echo "$JOINT_MSG" | grep -A1 "velocity:" | head -20
else
    echo -e "  ${RED}[ERROR]${NC} 无法获取 /joint_states"
fi
echo ""

# 5. 检查 TF
echo "--- 5. 检查 TF 树 ---"
if timeout 2 ros2 run tf2_tools view_frames 2>/dev/null; then
    echo -e "  ${GREEN}[OK]${NC} TF 树已生成 (frames.pdf)"
else
    echo "  检查关键 TF:"
    for frame in odom body_link; do
        if timeout 1 ros2 topic echo /tf --once 2>/dev/null | grep -q "$frame"; then
            echo -e "    ${GREEN}[OK]${NC} $frame"
        else
            echo -e "    ${RED}[MISSING]${NC} $frame"
        fi
    done
fi
echo ""

echo "=========================================="
echo "  诊断完成"
echo "=========================================="
echo ""
echo "提示: 如果要在 teleop 运行时测试，请在另一个终端按住 'i' 键"
echo "       然后重新运行此脚本查看数据流。"
