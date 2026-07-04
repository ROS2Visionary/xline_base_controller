#!/bin/bash
# XLine ROS 节点系统服务安装脚本
# 用法（目标机器上以 root 执行）:
#   sudo bash install_ros_services.sh
#
# 目录约定:
#   ~/xline_ws/                ← ROS工作空间根目录（DEPLOY_DIR）
#   ~/xline_ws/_xline_data_dir ← 数据目录

set -e

# ==============================================================
# 配置（按实际环境修改）
# ==============================================================
ROS_DISTRO="${ROS_DISTRO:-humble}"                    # ROS2 版本
XLINE_USER="${SUDO_USER:-xline}"                      # 运行服务的系统用户
XLINE_HOME="/home/${XLINE_USER}"                      # 用户主目录
DEPLOY_DIR="${XLINE_HOME}/xline_ws"                   # ROS工作空间根目录
XLINE_WS_ROOT="${XLINE_HOME}/xline_ws/_xline_data_dir" # 数据目录

# 工作空间 setup.bash 路径（source 后 ros2 命令可用）
# 若工作空间结构不同，修改此变量
ROS_WS_SETUP="${DEPLOY_DIR}/install/setup.bash"

SYSTEMD_DIR="/etc/systemd/system"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ==============================================================
# 检查权限
# ==============================================================
if [ "$(id -u)" -ne 0 ]; then
    echo "[ERROR] 请以 root 身份运行: sudo bash $0"
    exit 1
fi

echo "======================================"
echo "    XLine ROS 节点系统服务安装"
echo "======================================"
echo "  部署目录:        $DEPLOY_DIR"
echo "  ROS2 版本:       $ROS_DISTRO"
echo "  运行用户:        $XLINE_USER"
echo "  数据目录:        $XLINE_WS_ROOT"
echo "  工作空间 setup:  $ROS_WS_SETUP"
echo "======================================"
echo ""

# ==============================================================
# 停止并清理旧服务（如已安装）
# ==============================================================
if systemctl is-active --quiet xline-ros 2>/dev/null; then
    echo "停止旧服务: xline-ros"
    systemctl stop xline-ros
fi
if systemctl is-enabled --quiet xline-ros 2>/dev/null; then
    systemctl disable xline-ros
fi

# ==============================================================
# 安装看门狗脚本和节点配置文件
# ==============================================================
echo "安装看门狗脚本..."
cp "${SCRIPT_DIR}/xline_ros_watchdog.sh" "${DEPLOY_DIR}/xline_ros_watchdog.sh"
chmod +x "${DEPLOY_DIR}/xline_ros_watchdog.sh"
echo "已安装: ${DEPLOY_DIR}/xline_ros_watchdog.sh"

# 节点配置文件：若目标已存在则不覆盖（保留用户自定义）
if [ -f "${DEPLOY_DIR}/ros_nodes.conf" ]; then
    echo "ros_nodes.conf 已存在，跳过覆盖（保留现有配置）"
    echo "  如需重置默认配置: cp ${SCRIPT_DIR}/ros_nodes.conf ${DEPLOY_DIR}/ros_nodes.conf"
else
    # 替换配置文件中的路径占位符
    sed \
        -e "s|XLINE_WS_SETUP_PATH|${ROS_WS_SETUP}|g" \
        -e "s|XLINE_WS_ROOT_VALUE|${XLINE_WS_ROOT}|g" \
        "${SCRIPT_DIR}/ros_nodes.conf" > "${DEPLOY_DIR}/ros_nodes.conf"
    echo "已安装: ${DEPLOY_DIR}/ros_nodes.conf"
fi

# ==============================================================
# 生成并安装 systemd unit 文件（替换占位符）
# ==============================================================
echo "安装 systemd unit 文件..."
sed \
    -e "s|XLINE_ROS_DISTRO|${ROS_DISTRO}|g" \
    -e "s|XLINE_HOME|${XLINE_HOME}|g" \
    -e "s|XLINE_USER|${XLINE_USER}|g" \
    -e "s|DEPLOY_DIR|${DEPLOY_DIR}|g" \
    "${SCRIPT_DIR}/xline-ros.service" > "${SYSTEMD_DIR}/xline-ros.service"
echo "已安装: ${SYSTEMD_DIR}/xline-ros.service"

# ==============================================================
# 确保部署目录归属正确
# ==============================================================
chown "${XLINE_USER}:${XLINE_USER}" "${DEPLOY_DIR}/xline_ros_watchdog.sh" 2>/dev/null || true

# ==============================================================
# 配置免密 sudo（允许用户在 .bashrc 里切换开机自启，无需输密码）
# ==============================================================
echo "配置 sudo 免密规则..."
SUDOERS_FILE="/etc/sudoers.d/xline-ros-autostart"
cat > "$SUDOERS_FILE" << EOF
# 允许 ${XLINE_USER} 在不输密码的情况下 enable/disable xline-ros
${XLINE_USER} ALL=(root) NOPASSWD: \\
    /usr/bin/systemctl enable xline-ros.service, \\
    /usr/bin/systemctl disable xline-ros.service
EOF
chmod 440 "$SUDOERS_FILE"
echo "已写入: $SUDOERS_FILE"

# ==============================================================
# 将 .bashrc 控制片段追加到用户的 ~/.bashrc
# ==============================================================
BASHRC_FILE="${XLINE_HOME}/.bashrc"
BASHRC_MARKER="# XLine ROS 节点开机自启控制"

if grep -q "$BASHRC_MARKER" "$BASHRC_FILE" 2>/dev/null; then
    echo ".bashrc 已包含 XLine ROS 配置，跳过追加"
else
    echo "" >> "$BASHRC_FILE"
    cat "${SCRIPT_DIR}/bashrc_snippet_ros.sh" >> "$BASHRC_FILE"
    echo "已追加 XLine ROS 配置到: $BASHRC_FILE"
fi

# ==============================================================
# 启用并启动服务
# ==============================================================
echo ""
echo "启用并启动服务..."
systemctl daemon-reload
systemctl enable xline-ros.service
systemctl start xline-ros.service

# ==============================================================
# 状态检查
# ==============================================================
echo ""
echo "======================================"
echo "服务状态"
echo "======================================"
systemctl status xline-ros.service --no-pager -l || true

echo ""
echo "======================================"
echo "安装完成！"
echo ""
echo "常用管理命令:"
echo "  查看日志:     journalctl -u xline-ros -f"
echo "  看门狗日志:   tail -f ${DEPLOY_DIR}/logs/ros_watchdog.log"
echo "  重启服务:     systemctl restart xline-ros"
echo "  停止服务:     systemctl stop xline-ros"
echo "  开机禁用:     systemctl disable xline-ros"
echo ""
echo "增删/启用/禁用节点:"
echo "  编辑 ${DEPLOY_DIR}/ros_nodes.conf"
echo "  然后执行: systemctl restart xline-ros"
echo ""
echo "修改工作空间路径或数据目录:"
echo "  编辑 ${DEPLOY_DIR}/ros_nodes.conf 中的 [settings] 区块"
echo "  然后执行: systemctl restart xline-ros"
echo "======================================"
