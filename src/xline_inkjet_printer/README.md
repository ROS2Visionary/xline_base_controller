XLINE Inkjet Printer (ROS 2, Python)

概述
- 包名：`xline_inkjet_printer`，节点入口：`inkjet_printer_node`
- 功能：管理多路 TCP 连接（左/中/右打印机），支持断线重连与配置热更新。

配置文件（单文件，多 section）
- 路径：`src/xline_inkjet_printer/xline_inkjet_printer/config/printers.yaml`
- 结构：
  - `global_defaults`：全局默认（`timeout`、`reconnect_times`、`reconnect_interval`）
  - `connections`：以连接名为 key（`printer_left`、`printer_center`、`printer_right`），每项可覆盖默认并可 `enabled: true/false`

示例
global_defaults:
  timeout: 3
  reconnect_times: 0
  reconnect_interval: 2.0

connections:
  printer_left:
    ip: 192.168.0.24
    port: 6000
    enabled: true
  printer_center:
    ip: 192.168.0.25
    port: 6001
    enabled: true
  printer_right:
    ip: 192.168.0.26
    port: 6002
    enabled: true

运行
1) 构建（建议 symlink 安装以便热更新 yaml）
   colcon build --packages-select xline_inkjet_printer --symlink-install
   source install/setup.bash

2) 启动节点
   ros2 run xline_inkjet_printer inkjet_printer_node

3) 在线修改 `printers.yaml`
- 节点每秒自动重载；修改 IP/端口会触发重连；`reconnect_times: 0` 表示无限重连；将某连接 `enabled: false` 可暂停该路连接管理。

说明
- 旧的多文件配置（`printer.yaml`/`printer_aux*.yaml`）已移除，改用 `printers.yaml` 统一管理。
