#!/bin/bash
# XLine ROS 节点系统服务卸载脚本
# 用法（目标机器上以 root 执行）:
#   sudo bash uninstall_ros_services.sh

set -e

# ==============================================================
# 配置（需与安装时保持一致）
# ==============================================================
XLINE_USER="${SUDO_USER:-xline}"
XLINE_HOME="/home/${XLINE_USER}"
DEPLOY_DIR="${XLINE_HOME}/xline_ws"

SYSTEMD_DIR="/etc/systemd/system"
SUDOERS_FILE="/etc/sudoers.d/xline-ros-autostart"
BASHRC_FILE="${XLINE_HOME}/.bashrc"
BASHRC_MARKER="# XLine ROS 节点开机自启控制"

# ==============================================================
# 检查权限
# ==============================================================
if [ "$(id -u)" -ne 0 ]; then
    echo "[ERROR] 请以 root 身份运行: sudo bash $0"
    exit 1
fi

echo "======================================"
echo "    XLine ROS 节点系统服务卸载"
echo "======================================"
echo "  部署目录:   $DEPLOY_DIR"
echo "  运行用户:   $XLINE_USER"
echo "======================================"
echo ""

# ==============================================================
# 停止并禁用服务
# ==============================================================
if systemctl is-active --quiet xline-ros 2>/dev/null; then
    echo "停止服务: xline-ros"
    systemctl stop xline-ros
fi
if systemctl is-enabled --quiet xline-ros 2>/dev/null; then
    echo "禁用服务: xline-ros"
    systemctl disable xline-ros
fi

# ==============================================================
# 删除 systemd unit 文件
# ==============================================================
UNIT_FILE="${SYSTEMD_DIR}/xline-ros.service"
if [ -f "$UNIT_FILE" ]; then
    rm -f "$UNIT_FILE"
    echo "已删除: $UNIT_FILE"
fi
systemctl daemon-reload

# ==============================================================
# 删除看门狗脚本
# ==============================================================
WATCHDOG="${DEPLOY_DIR}/xline_ros_watchdog.sh"
if [ -f "$WATCHDOG" ]; then
    rm -f "$WATCHDOG"
    echo "已删除: $WATCHDOG"
fi

# ==============================================================
# 删除 sudoers 规则
# ==============================================================
if [ -f "$SUDOERS_FILE" ]; then
    rm -f "$SUDOERS_FILE"
    echo "已删除: $SUDOERS_FILE"
fi

# ==============================================================
# 从 ~/.bashrc 中移除 XLine ROS 控制片段
# ==============================================================
if grep -q "$BASHRC_MARKER" "$BASHRC_FILE" 2>/dev/null; then
    python3 - "$BASHRC_FILE" "$BASHRC_MARKER" << 'PYEOF'
import sys

bashrc_path = sys.argv[1]
marker = sys.argv[2]

with open(bashrc_path, 'r') as f:
    lines = f.readlines()

start = next((i for i, l in enumerate(lines) if marker in l), None)
if start is None:
    sys.exit(0)

if start > 0 and lines[start - 1].startswith('# ='):
    start -= 1
while start > 0 and lines[start - 1].strip() == '':
    start -= 1

end = start
for i in range(start, len(lines)):
    if '_xline_ros_sync_autostart' in lines[i] and not lines[i].strip().startswith('_xline_ros_sync_autostart()'):
        end = i + 1
        break

with open(bashrc_path, 'w') as f:
    f.writelines(lines[:start] + lines[end:])

print(f"已从 {bashrc_path} 移除 XLine ROS 配置（行 {start+1}–{end}）")
PYEOF
else
    echo ".bashrc 中未找到 XLine ROS 配置，跳过"
fi

echo ""
echo "======================================"
echo "卸载完成！"
echo "注意: 部署目录 $DEPLOY_DIR 及其数据未删除。"
echo "======================================"
