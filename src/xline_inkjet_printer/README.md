# XLINE Inkjet Printer (ROS 2, Python)

高性能、基于 asyncio 的喷墨打印机控制节点。

## 📋 目录

- [概述](#概述)
- [特性](#特性)
- [架构](#架构)
- [配置](#配置)
- [安装](#安装)
- [使用](#使用)
- [ROS 2 接口](#ros-2-接口)
- [性能监控](#性能监控)
- [故障排除](#故障排除)

---

## 概述

`xline_inkjet_printer` 是一个专业的 ROS 2 打印机控制包，用于管理多路喷墨打印机的 TCP 连接。

**包名**: `xline_inkjet_printer`
**节点名**: `async_inkjet_printer_node` (推荐) / `inkjet_printer_node` (旧版)
**版本**: v2.0 (基于 asyncio 重构)

---

## 特性

### ✨ 核心特性

- **异步 I/O**: 基于 Python asyncio，高性能、低延迟
- **多路连接**: 支持同时管理 3 路独立 TCP 连接（左/中/右打印机）
- **自动重连**: 智能断线重连机制，可配置重连次数和间隔
- **配置热更新**: 无需重启节点即可更新配置（每秒自动检测）
- **协议抽象**: 支持文本协议和二进制协议，易于扩展
- **完整统计**: 连接成功率、在线时长、数据流量、响应时间等

### 🔧 高级特性

- **配置验证**: 自动验证 IP、端口、参数范围
- **独立启停**: 每个打印机可独立启用/禁用
- **ROS 2 服务**: 提供完整的服务接口（打印、状态查询、统计）
- **状态发布**: 实时发布连接状态和统计信息
- **线程安全**: 所有操作线程安全，支持并发调用

---

## 架构

### 模块结构

```
xline_inkjet_printer/
├── protocol.py              # 协议抽象层
│   ├── PrinterProtocol      # 协议基类
│   ├── InkjetProtocol       # 文本协议实现
│   └── BinaryInkjetProtocol # 二进制协议实现
├── config_validator.py      # 配置验证工具
├── connection_stats.py      # 性能统计模块
├── async_tcp_client.py      # 异步 TCP 客户端
├── async_inkjet_node.py     # 异步 ROS 2 节点 (推荐)
├── tcp_client.py            # 同步 TCP 客户端 (旧版)
└── inkjet_node.py           # 同步 ROS 2 节点 (旧版)
```

### 协议层次

```
┌─────────────────────────────────┐
│   AsyncInkjetPrinterNode        │  ROS 2 节点层
│   (ROS 2 服务、话题、参数)      │
└─────────────┬───────────────────┘
              │
┌─────────────▼───────────────────┐
│   AsyncTcpClient                │  TCP 客户端层
│   (连接管理、数据收发)          │
└─────────────┬───────────────────┘
              │
┌─────────────▼───────────────────┐
│   PrinterProtocol               │  协议层
│   (命令编码、响应解析)          │
└─────────────────────────────────┘
```

---

## 配置

### 配置文件

**路径**: `src/xline_inkjet_printer/xline_inkjet_printer/config/printers.yaml`

### 配置结构

```yaml
# 全局默认参数
global_defaults:
  timeout: 3              # 连接超时时间（秒）
  reconnect_times: 0      # 最大重连次数（0 = 无限重连）
  reconnect_interval: 2.0 # 重连间隔（秒）

# 各个打印机连接配置
connections:
  printer_left:
    ip: 192.168.0.23      # IP 地址
    port: 6000            # 端口号
    enabled: true         # 是否启用
    # 可选：覆盖全局参数
    timeout: 5
    reconnect_times: 10

  printer_center:
    ip: 192.168.0.24
    port: 6000
    enabled: true

  printer_right:
    ip: 192.168.0.26
    port: 6000
    enabled: false        # 禁用此连接
```

### 参数说明

| 参数 | 类型 | 范围 | 默认值 | 说明 |
|------|------|------|--------|------|
| `ip` | string | 有效的IP地址或主机名 | - | 打印机地址 |
| `port` | int | 1-65535 | - | TCP 端口 |
| `timeout` | float | 0.1-300.0 | 3.0 | 连接超时（秒） |
| `reconnect_times` | int | 0-1000 | 0 | 最大重连次数（0=无限） |
| `reconnect_interval` | float | 0.1-3600.0 | 2.0 | 重连间隔（秒） |
| `enabled` | bool | true/false | true | 是否启用 |

---

## 安装

### 1. 构建包

```bash
cd /root/xline_ws/xline_base_controller
colcon build --packages-select xline_inkjet_printer --symlink-install
source install/setup.bash
```

**注意**: 使用 `--symlink-install` 可以实现配置文件的热更新。

### 2. 验证安装

```bash
# 检查节点是否可用
ros2 pkg executables xline_inkjet_printer

# 应该看到：
# xline_inkjet_printer async_inkjet_printer_node
# xline_inkjet_printer inkjet_printer_node
```

---

## 使用

### 启动节点

#### 方式 1: 使用默认配置

```bash
ros2 run xline_inkjet_printer async_inkjet_printer_node
```

#### 方式 2: 自定义参数

```bash
ros2 run xline_inkjet_printer async_inkjet_printer_node \
  --ros-args \
  -p config_file:=printers.yaml \
  -p protocol_type:=inkjet \
  -p status_publish_rate:=5.0
```

#### 方式 3: 使用 Launch 文件

创建 `launch/inkjet_printer.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xline_inkjet_printer',
            executable='async_inkjet_printer_node',
            name='inkjet_printer_node',
            output='screen',
            parameters=[{
                'config_file': 'printers.yaml',
                'protocol_type': 'inkjet',
                'status_publish_rate': 5.0,
            }]
        )
    ])
```

启动：

```bash
ros2 launch xline_inkjet_printer inkjet_printer.launch.py
```

---

## ROS 2 接口

### 话题 (Topics)

#### `/printer_status` (std_msgs/String)

发布所有打印机的状态信息（JSON 格式），频率可配置（默认 1Hz）。

**示例输出**:

```json
{
  "printer_left": {
    "connected": true,
    "auto_connect": true,
    "enabled": true,
    "is_online": true,
    "status": "已连接",
    "device_id": 0,
    "ink_level": 85
  },
  "printer_center": {
    "connected": true,
    "auto_connect": true,
    "enabled": true,
    "is_online": true,
    "status": "已连接",
    "device_id": 0,
    "ink_level": 72
  },
  "printer_right": {
    "connected": false,
    "auto_connect": false,
    "enabled": false,
    "is_online": false,
    "status": "已断开",
    "device_id": 0,
    "ink_level": 0
  }
}
```

**字段说明**:
- `connected`: TCP 连接状态
- `auto_connect`: 自动重连开关
- `enabled`: 功能启用状态
- `is_online`: IP 在线状态（Ping 检测，TCP 连接时不检测）
- `status`: 人类可读的状态描述（中文）
- `device_id`: 设备ID（0-255）
- `ink_level`: 墨量余量百分比（0-100，0表示未查询或查询失败）

**`is_online` 字段说明**:
- 检测机制：TCP 连接时不执行 Ping，未连接时每 2 秒检测一次
- 用于网络诊断：
  - `is_online=true, connected=false`：网络通但 TCP 失败（端口/防火墙问题）
  - `is_online=false, connected=false`：网络不通或主机离线

**订阅示例**:

```bash
ros2 topic echo /printer_status
```

**详细文档**: 请参阅 [TOPIC_printer_status.md](./TOPIC_printer_status.md)

---

### 服务 (Services)

#### 1. 打印命令服务

发送打印命令到指定打印机。

**服务列表**:
- `/printer_left/print` (std_srvs/Trigger)
- `/printer_center/print` (std_srvs/Trigger)
- `/printer_right/print` (std_srvs/Trigger)

**调用示例**:

```bash
# 左侧打印机打印
ros2 service call /printer_left/print std_srvs/srv/Trigger

# 响应示例：
# success: True
# message: "打印命令已发送"
```

#### 2. 状态查询服务

查询指定打印机的详细状态和统计信息。

**服务列表**:
- `/printer_left/status` (std_srvs/Trigger)
- `/printer_center/status` (std_srvs/Trigger)
- `/printer_right/status` (std_srvs/Trigger)

**调用示例**:

```bash
ros2 service call /printer_left/status std_srvs/srv/Trigger
```

**响应示例**:

```json
{
  "success": true,
  "message": {
    "name": "printer_left",
    "connected": true,
    "enabled": true,
    "status": "已连接",
    "stats": {
      "connections": {
        "total_connects": 5,
        "total_disconnects": 2,
        "total_failures": 1,
        "consecutive_failures": 0,
        "success_rate": "83.33%"
      },
      "data": {
        "bytes_sent": 1245,
        "bytes_received": 850,
        "messages_sent": 15,
        "messages_received": 12
      },
      "timing": {
        "total_uptime_seconds": "3600.50",
        "current_session_seconds": "1200.30",
        "uptime_percentage": "98.50%",
        "first_connect": "2025-10-15 10:30:00",
        "last_connect": "2025-10-15 11:45:00",
        "last_disconnect": "2025-10-15 11:30:00"
      },
      "performance": {
        "avg_response_time_ms": "12.35",
        "min_response_time_ms": "5.20",
        "max_response_time_ms": "45.80",
        "response_samples": 100
      }
    }
  }
}
```

#### 3. 获取所有统计信息

**服务**: `/get_stats` (std_srvs/Trigger)

获取所有打印机的统计信息。

```bash
ros2 service call /get_stats std_srvs/srv/Trigger
```

#### 4. 重置统计信息

**服务**: `/reset_stats` (std_srvs/Trigger)

重置所有打印机的统计计数器。

```bash
ros2 service call /reset_stats std_srvs/srv/Trigger
```

---

## 性能监控

### 实时监控

#### 方法 1: 订阅状态话题

```bash
ros2 topic echo /printer_status
```

#### 方法 2: 查询统计服务

```bash
# 单个打印机
ros2 service call /printer_left/status std_srvs/srv/Trigger

# 所有打印机
ros2 service call /get_stats std_srvs/srv/Trigger
```

### 关键指标

| 指标 | 说明 | 正常范围 |
|------|------|----------|
| 连接成功率 | `success_rate` | > 95% |
| 在线时长百分比 | `uptime_percentage` | > 99% |
| 平均响应时间 | `avg_response_time_ms` | < 50ms |
| 连续失败次数 | `consecutive_failures` | 0 |

### 告警阈值建议

```python
# 示例监控脚本
if stats['success_rate'] < 0.95:
    logger.warning("连接成功率低于 95%")

if stats['consecutive_failures'] > 5:
    logger.error("连续失败超过 5 次")

if stats['avg_response_time_ms'] > 100:
    logger.warning("平均响应时间过长")
```

---

## 故障排除

### 常见问题

#### 1. 连接失败

**症状**: 日志显示 "连接失败" 或 "连接超时"

**解决方案**:
1. 检查打印机 IP 和端口是否正确
   ```bash
   ping 192.168.0.23
   nc -zv 192.168.0.23 6000
   ```
2. 检查网络连通性和防火墙
3. 验证打印机是否在线
4. 增加 `timeout` 参数值

#### 2. 配置不生效

**症状**: 修改配置文件后没有变化

**解决方案**:
1. 确保使用 `--symlink-install` 构建
2. 检查配置文件路径是否正确
3. 查看节点日志中的配置加载信息
4. 配置热更新有 1 秒延迟（正常）

#### 3. 达到最大重连次数

**症状**: 日志显示 "已达到最大重连次数"

**解决方案**:
1. 修改配置文件中的 `reconnect_times: 0`（无限重连）
2. 增加 `reconnect_interval` 避免频繁重连
3. 检查并修复网络问题后，重启节点

#### 4. 性能问题

**症状**: 响应时间长或丢包

**解决方案**:
1. 检查网络延迟
   ```bash
   ping -c 100 192.168.0.23
   ```
2. 减少 `status_publish_rate` 参数
3. 检查打印机负载
4. 查看统计信息中的响应时间分布

---

## 协议扩展

### 添加自定义协议

1. **创建协议类**:

```python
# my_protocol.py
from xline_inkjet_printer.protocol import PrinterProtocol, CommandType

class MyCustomProtocol(PrinterProtocol):
    def get_protocol_name(self) -> str:
        return "MyProtocol-v1.0"

    def encode_command(self, cmd_type: CommandType, **kwargs) -> bytes:
        # 实现编码逻辑
        pass

    def decode_response(self, data: bytes) -> dict:
        # 实现解码逻辑
        pass
```

2. **注册协议**:

```python
from xline_inkjet_printer.protocol import ProtocolFactory
from my_protocol import MyCustomProtocol

ProtocolFactory.register_protocol('my_protocol', MyCustomProtocol)
```

3. **使用自定义协议**:

```bash
ros2 run xline_inkjet_printer async_inkjet_printer_node \
  --ros-args -p protocol_type:=my_protocol
```

---

## 迁移指南

### 从旧版本迁移

如果你正在使用旧的 `inkjet_printer_node`，迁移到新的 `async_inkjet_printer_node`：

#### 步骤 1: 更新配置文件

配置文件格式保持兼容，无需修改。

#### 步骤 2: 更新启动命令

```bash
# 旧版本
ros2 run xline_inkjet_printer inkjet_printer_node

# 新版本
ros2 run xline_inkjet_printer async_inkjet_printer_node
```

#### 步骤 3: 更新代码

服务接口保持兼容，但响应格式更丰富（包含统计信息）。

---

## 性能对比

| 特性 | 旧版本 (sync) | 新版本 (async) |
|------|--------------|---------------|
| I/O 模型 | 同步阻塞 | 异步非阻塞 |
| 并发性能 | 低（多线程） | 高（协程） |
| 内存占用 | 较高 | 较低 |
| CPU 使用率 | 较高 | 较低 |
| 响应时间 | 10-50ms | 5-20ms |
| 协议支持 | 硬编码 | 抽象可扩展 |
| 性能统计 | 无 | 完整 |
| 配置验证 | 基础 | 严格 |

---

## 开发者信息

**维护者**: xline
**邮箱**: maintainer@example.com
**许可证**: Proprietary

---

## 更新日志

### v2.0.0 (2025-10-15)

- ✨ 全新的 asyncio 架构
- ✨ 协议抽象层支持
- ✨ 完整的性能统计
- ✨ 配置验证系统
- ✨ ROS 2 服务接口
- 📝 完善的文档

### v1.0.0

- 基础的 TCP 连接管理
- 简单的重连机制
- 配置热更新

---

## 贡献指南

欢迎提交 Issue 和 Pull Request！

---

## 常用命令速查

```bash
# 构建
colcon build --packages-select xline_inkjet_printer --symlink-install
source install/setup.bash

# 启动节点
ros2 run xline_inkjet_printer async_inkjet_printer_node

# 查看状态
ros2 topic echo /printer_status

# 发送打印命令
ros2 service call /printer_left/print std_srvs/srv/Trigger

# 查询详细状态
ros2 service call /printer_left/status std_srvs/srv/Trigger

# 获取所有统计
ros2 service call /get_stats std_srvs/srv/Trigger

# 重置统计
ros2 service call /reset_stats std_srvs/srv/Trigger

# 查看节点信息
ros2 node info /inkjet_printer_node

# 查看所有服务
ros2 service list
```
