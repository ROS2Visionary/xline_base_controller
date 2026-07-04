#!/bin/bash
# xline ROS节点看门狗
# 从 ros_nodes.conf（INI格式）读取路径配置和节点列表
# 监控每个节点进程，崩溃时自动重启
# 由 systemd xline-ros.service 托管，本脚本永不主动退出

set -uo pipefail

# ==============================================================
# 基础路径
# ==============================================================
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# 节点配置文件路径，可通过环境变量覆盖
ROS_NODES_CONF="${ROS_NODES_CONF:-$SCRIPT_DIR/ros_nodes.conf}"

# ==============================================================
# 基础配置
# ==============================================================
CHECK_INTERVAL=10      # 进程检查间隔（秒）
STARTUP_GRACE=30       # 首次启动后等待所有节点初始化的时间（秒）
RESTART_COOLDOWN=5     # 单个节点重启后冷却时间（秒）
STARTUP_CHECK_DELAY=3  # 启动每个节点后等待检测是否立即退出的时间（秒）
LOG_FILE="$SCRIPT_DIR/logs/ros_watchdog.log"

mkdir -p "$SCRIPT_DIR/logs"

# ==============================================================
# 工具函数
# ==============================================================
log() {
    local msg="[$(date '+%Y-%m-%d %H:%M:%S')] $*"
    echo "$msg"
    echo "$msg" >> "$LOG_FILE"
}

pid_alive() {
    local pid="${1:-}"
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

kill_if_alive() {
    local pid="${1:-}"
    [ -z "$pid" ] && return 0
    kill "$pid" 2>/dev/null || true
    local i
    for i in 1 2 3; do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 1
    done
    kill -9 "$pid" 2>/dev/null || true
}

# ==============================================================
# INI 配置解析
# 单次遍历：先收集 [settings]，再收集节点定义
# ==============================================================
# 从 [settings] 读取的路径（未设则为空，留给后续赋默认值）
CFG_ROS_WS_SETUP=""
CFG_XLINE_WS_ROOT=""

declare -a NODE_ORDER=()
declare -A NODE_DEFS=()     # name -> "type pkg node"
declare -A NODE_DELAYS=()   # name -> delay seconds
declare -A NODE_PIDS=()     # name -> PID（运行时维护）

_trim() {
    local s="$1"
    s="${s#"${s%%[![:space:]]*}"}"
    s="${s%"${s##*[![:space:]]}"}"
    echo "$s"
}

_flush_node() {
    local sname="$1"
    [ "$sname" = "settings" ] || [ -z "$sname" ] && return
    [ "${_n_enabled:-false}" != "true" ] && return
    if [ -z "${_n_pkg:-}" ] || [ -z "${_n_node:-}" ]; then
        log "[警告] 节点 [$sname] 缺少 pkg 或 node 字段，已跳过"
        return
    fi
    NODE_ORDER+=("$sname")
    NODE_DEFS["$sname"]="${_n_type:-run} ${_n_pkg} ${_n_node}"
    NODE_DELAYS["$sname"]="${_n_delay:-0}"
    NODE_PIDS["$sname"]=""
}

load_config() {
    if [ ! -f "$ROS_NODES_CONF" ]; then
        log "[错误] 配置文件未找到: $ROS_NODES_CONF"
        exit 1
    fi

    local section=""
    _n_enabled="" ; _n_type="" ; _n_pkg="" ; _n_node="" ; _n_delay=""

    while IFS= read -r line || [ -n "$line" ]; do
        # 跳过注释行和空行
        [[ "$line" =~ ^[[:space:]]*# ]] && continue
        [[ -z "${line//[[:space:]]/}" ]] && continue

        # 区块头 [name]
        if [[ "$line" =~ ^\[([[:alnum:]_-]+)\]$ ]]; then
            _flush_node "$section"
            section="${BASH_REMATCH[1]}"
            _n_enabled="" ; _n_type="" ; _n_pkg="" ; _n_node="" ; _n_delay=""
            continue
        fi

        # key = value
        if [[ "$line" =~ ^([^=]+)=(.*)$ ]]; then
            local key val
            key="$(_trim "${BASH_REMATCH[1]}")"
            val="$(_trim "${BASH_REMATCH[2]}")"

            case "$section" in
                settings)
                    case "$key" in
                        ros_ws_setup)  CFG_ROS_WS_SETUP="$val" ;;
                        xline_ws_root) CFG_XLINE_WS_ROOT="$val" ;;
                    esac
                    ;;
                *)
                    case "$key" in
                        enabled) _n_enabled="$val" ;;
                        type)    _n_type="$val" ;;
                        pkg)     _n_pkg="$val" ;;
                        node)    _n_node="$val" ;;
                        delay)   _n_delay="$val" ;;
                    esac
                    ;;
            esac
        fi
    done < "$ROS_NODES_CONF"

    _flush_node "$section"
}

# ==============================================================
# 读取配置（先解析，再 source ROS 环境）
# ==============================================================
load_config

# 优先级：环境变量 > 配置文件 > 内置默认
ROS_WS_SETUP="${ROS_WS_SETUP:-${CFG_ROS_WS_SETUP:-$SCRIPT_DIR/install/setup.bash}}"
export XLINE_WS_ROOT="${XLINE_WS_ROOT:-${CFG_XLINE_WS_ROOT:-$SCRIPT_DIR}}"

# ==============================================================
# Source ROS 环境
# ==============================================================
set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f "$ROS_WS_SETUP" ]; then
    source "$ROS_WS_SETUP"
else
    log "[警告] 工作空间 setup.bash 未找到: $ROS_WS_SETUP"
fi
set -u

# ==============================================================
# 通用节点启停函数
# ==============================================================
start_node() {
    local name="$1"
    local def="${NODE_DEFS[$name]}"
    local type pkg node_or_launch
    read -r type pkg node_or_launch <<< "$def"

    kill_if_alive "${NODE_PIDS[$name]:-}"

    if [ "$type" = "launch" ]; then
        ros2 launch "$pkg" "$node_or_launch" >/dev/null 2>&1 &
    else
        ros2 run "$pkg" "$node_or_launch" >/dev/null 2>&1 &
    fi

    NODE_PIDS["$name"]=$!
    log "[$name] 已启动 PID=${NODE_PIDS[$name]}  (ros2 $type $pkg $node_or_launch)"
}

# ==============================================================
# 退出清理（systemd stop / SIGTERM）
# ==============================================================
cleanup() {
    trap '' SIGTERM SIGINT
    log "收到退出信号，停止所有 ROS 节点..."
    for name in "${NODE_ORDER[@]}"; do
        kill_if_alive "${NODE_PIDS[$name]:-}"
    done
    wait 2>/dev/null || true
    log "所有 ROS 节点已停止"
    exit 0
}
trap cleanup SIGTERM SIGINT

# ==============================================================
# 首次启动
# ==============================================================
log "======== xline ROS 节点看门狗启动 ========"
log "ROS_DISTRO=$ROS_DISTRO"
log "ROS_WS_SETUP=$ROS_WS_SETUP"
log "XLINE_WS_ROOT=$XLINE_WS_ROOT"
log "ROS_NODES_CONF=$ROS_NODES_CONF"

if [ ${#NODE_ORDER[@]} -eq 0 ]; then
    log "[错误] 没有任何启用的节点，请检查配置文件: $ROS_NODES_CONF"
    exit 1
fi

log "已启用节点（${#NODE_ORDER[@]} 个）: ${NODE_ORDER[*]}"

for name in "${NODE_ORDER[@]}"; do
    start_node "$name"

    # 等待 STARTUP_CHECK_DELAY 秒检测节点是否立即退出，确保后续节点不受影响
    sleep "$STARTUP_CHECK_DELAY"
    if ! pid_alive "${NODE_PIDS[$name]:-}"; then
        log "[警告] $name 启动后 ${STARTUP_CHECK_DELAY}s 内已退出，跳过等待，继续启动后续节点"
        NODE_PIDS["$name"]=""
        continue
    fi

    delay="${NODE_DELAYS[$name]:-0}"
    remaining=$(( delay - STARTUP_CHECK_DELAY ))
    if [ "$remaining" -gt 0 ] 2>/dev/null; then
        log "等待 ${remaining}s（$name 启动后延迟）..."
        sleep "$remaining"
    fi
done

log "等待 ${STARTUP_GRACE}s 让所有节点初始化完成..."
sleep "$STARTUP_GRACE"

# 启动完成后汇总各节点状态
log "--- 启动状态汇总 ---"
for name in "${NODE_ORDER[@]}"; do
    if pid_alive "${NODE_PIDS[$name]:-}"; then
        log "  [OK]   $name  PID=${NODE_PIDS[$name]}"
    else
        log "  [FAIL] $name  未运行"
    fi
done
log "--- 汇总结束 ---"

# ==============================================================
# 看门狗主循环
# ==============================================================
log "看门狗开始监控，检查间隔 ${CHECK_INTERVAL}s"

while true; do
    sleep "$CHECK_INTERVAL"

    for name in "${NODE_ORDER[@]}"; do
        if ! pid_alive "${NODE_PIDS[$name]:-}"; then
            log "[WATCHDOG] $name 崩溃（PID=${NODE_PIDS[$name]:-?}），重启..."
            start_node "$name"
            sleep "$RESTART_COOLDOWN"
        fi
    done
done
