#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# CycloneDDS unicast 发现环境配置（公共函数）
#
# 被 dispatch.sh / rviz_multi.sh / rviz_remote.sh / teleop.sh source。
# 提供 setup_cyclonedds_uri() 生成禁 multicast + unicast peers 的临时 xml，
# 并 export CYCLONEDDS_URI。同机节点靠 loopback 自动发现，互不需 peer。
#
# 用法:
#   source "${PROJECT_DIR}/scripts/launch/_dds_env.sh"
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp     # 各脚本自行 export
#   setup_cyclonedds_uri "${CYCLONE_PEERS[@]}"       # 空数组=仅本机 loopback
#   trap '...; cyclonedds_cleanup' EXIT              # 退出时清理临时 xml
# ─────────────────────────────────────────────────────────────────────

# 全局：临时 cyclonedds xml 路径（由 setup_cyclonedds_uri 设置，cyclonedds_cleanup 清理）
CYCLONE_TMP_XML="${CYCLONE_TMP_XML:-}"

# setup_cyclonedds_uri [peer1] [peer2] ...
# 生成 allowMulticast=false + unicast peers 的 cyclonedds xml，export CYCLONEDDS_URI。
# peer 格式支持 host 或 host:port（端口会被剥除，CycloneDDS peer 仅需 host，端口由 domain id 决定）。
# 无参数时不设 CYCLONEDDS_URI，CycloneDDS 走默认（本机 loopback 发现）。
setup_cyclonedds_uri() {
    unset ROS_DISCOVERY_SERVER   # 清除旧 FastDDS Discovery Server 语义，避免残留

    local template="${PROJECT_DIR}/config/cyclonedds.xml.template"
    # 始终包含 loopback peer：CycloneDDS 在 allowMulticast=false 下不会自动 loopback 发现，
    # 必须显式声明 127.0.0.1，否则同机多 participant（如 dispatch 的数十个节点）无法互发现。
    local peers_block="<peer address=\"127.0.0.1\"/>"
    local p host

    for p in "$@"; do
        [ -n "$p" ] || continue
        host="${p%%:*}"   # 剥端口（容错 host:port 输入）
        [ -n "$host" ] && [ "$host" != "127.0.0.1" ] && peers_block+="<peer address=\"${host}\"/>"
    done

    if [ ! -f "$template" ]; then
        echo "ERROR: CycloneDDS 模板缺失: $template" >&2
        return 1
    fi

    CYCLONE_TMP_XML="$(mktemp /tmp/cyclonedds_XXXXXX.xml)"
    sed "s|{{CYCLONE_PEERS}}|${peers_block}|" "$template" > "$CYCLONE_TMP_XML"
    export CYCLONEDDS_URI="file://${CYCLONE_TMP_XML}"
}

# 清理临时 xml（脚本 trap/退出时调用）
cyclonedds_cleanup() {
    if [ -n "${CYCLONE_TMP_XML:-}" ] && [ -f "$CYCLONE_TMP_XML" ]; then
        rm -f "$CYCLONE_TMP_XML"
    fi
}
