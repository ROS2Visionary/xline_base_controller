# ==============================================================
# XLine ROS 节点开机自启控制
# 修改 XLINE_ROS_AUTOSTART 后执行 source ~/.bashrc 即可生效
#   true  → 开机自动启动 xline-ros（ROS节点）
#   false → 关闭开机自启（已运行的服务不受影响）
# ==============================================================
XLINE_ROS_AUTOSTART=true

_xline_ros_sync_autostart() {
    local current
    current=$(systemctl is-enabled xline-ros.service 2>/dev/null) || return 0

    local desired="${XLINE_ROS_AUTOSTART:-false}"

    if [ "$desired" = "true" ] && [ "$current" != "enabled" ]; then
        sudo systemctl enable xline-ros.service --quiet 2>/dev/null \
            && echo "[XLine ROS] 开机自启已开启"
    elif [ "$desired" != "true" ] && [ "$current" = "enabled" ]; then
        sudo systemctl disable xline-ros.service --quiet 2>/dev/null \
            && echo "[XLine ROS] 开机自启已关闭"
    fi
}

_xline_ros_sync_autostart
