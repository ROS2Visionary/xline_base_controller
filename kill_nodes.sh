#!/bin/bash

###############################################################################
# ROS2 节点终止脚本
# 用于终止启动脚本中启动的所有节点
###############################################################################

echo "======================================"
echo "正在终止 ROS2 节点..."
echo "======================================"

# 定义要终止的节点关键字
NODES=(
    "planner_node"
    "base_controller_node"
    "beiwei_imu_driver"
    "wheels_driver_node"
)

# 函数: 终止指定节点
kill_node() {
    local node_name=$1
    echo -n "终止 ${node_name}... "

    # 使用 pkill -f 匹配完整命令行
    if pkill -f "${node_name}"; then
        echo "✓"
    else
        echo "未找到或已停止"
    fi
}

# 终止所有节点
for node in "${NODES[@]}"; do
    kill_node "$node"
done

# 等待进程完全终止
echo ""
echo "等待进程终止..."
sleep 1

# 检查是否还有残留进程
echo ""
echo "检查残留进程:"
for node in "${NODES[@]}"; do
    if pgrep -f "${node}" > /dev/null; then
        echo "  ⚠️  ${node} 仍在运行，强制终止..."
        pkill -9 -f "${node}"
    else
        echo "  ✓ ${node} 已停止"
    fi
done

echo ""
echo "======================================"
echo "所有节点已终止完成"
echo "======================================"
