#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# 保存地图 (Save Map)
#
# 将当前 slam_toolbox 发布的 /map 保存为 PGM + YAML 文件
# 必须在 SLAM 建图运行中执行
#
# 用法:
#   ./scripts/launch/save_map.sh                    # 默认: maps/map
#   ./scripts/launch/save_map.sh -f maps/my_map     # 指定文件名 (无扩展名)
# ─────────────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 默认参数
MAP_NAME="${PROJECT_DIR}/maps/map"

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        -f)
            MAP_NAME="$2"
            shift 2
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [-f map_filename (无扩展名)]"
            exit 1
            ;;
    esac
done

# 确保 maps 目录存在
MAP_DIR="$(dirname "$MAP_NAME")"
mkdir -p "$MAP_DIR"

echo "=== 保存地图到 ${MAP_NAME} ==="
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" --ros-args -p use_sim_time:=false

echo "=== 地图已保存 ==="
ls -la "${MAP_NAME}".*
