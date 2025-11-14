# ROS 2 话题：`printer_status`

## 概述

`printer_status` 是由喷码机节点定时发布的状态话题，包含所有喷码机的实时状态信息。

---

## 基本信息

| 属性 | 值 |
|------|-----|
| **话题名称** | `/printer_status` |
| **消息类型** | `std_msgs/msg/String` |
| **发布节点** | `inkjet_printer_node` |
| **发布频率** | 1.0 Hz（默认，可通过参数 `status_publish_rate` 配置） |
| **消息格式** | JSON 字符串 |
| **QoS** | 默认（history: keep_last 10） |

---

## 消息内容

### 数据格式

消息的 `data` 字段为 JSON 格式字符串，包含三个喷码机（left/center/right）的状态信息。

### JSON 结构

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

---

## 字段说明

### 顶层字段

每个喷码机（`printer_left`、`printer_center`、`printer_right`）包含以下字段：

| 字段名 | 类型 | 范围/可选值 | 说明 |
|--------|------|------------|------|
| `connected` | boolean | `true` / `false` | TCP 连接状态：是否已连接到喷码机 |
| `auto_connect` | boolean | `true` / `false` | 自动连接开关：是否允许自动重连 |
| `enabled` | boolean | `true` / `false` | 功能启用状态：是否允许发送指令 |
| `is_online` | boolean | `true` / `false` | IP 在线状态：是否可 ping 通（TCP 连接时不检测）|
| `status` | string | 见下表 | 人类可读的状态描述（中文） |
| `device_id` | integer | 0-255 | 设备ID（用于多设备通信） |
| `ink_level` | integer | 0-100 | 墨量余量百分比（0=空，100=满） |

### `is_online` 字段详解

- **含义**：IP 地址是否在线（可 ping 通）
- **检测机制**：
  - TCP 连接成功时：**不执行** ping 检测（避免浪费资源）
  - TCP 未连接时：每隔 **2 秒** 执行一次 ping 检测
- **用途**：用于诊断网络连通性问题
  - `is_online=true, connected=false`：网络通但 TCP 连接失败（可能是端口或防火墙问题）
  - `is_online=false, connected=false`：网络不通或主机离线

### `status` 字段可能的值

| 值 | 说明 | 对应条件 |
|----|------|---------|
| `"已连接"` | 正常运行 | `connected=true` |
| `"已断开"` | TCP 未连接 | `connected=false` |
| `"重连中 (N/M)"` | 正在尝试重连 | 断线后自动重连中 |
| `"已放弃重连"` | 达到最大重连次数 | 已达到重连上限 |
| `"自动连接已禁用"` | 功能被禁用 | `auto_connect=false` |

### `ink_level` 字段详解

- **含义**：墨量余量百分比
- **取值范围**：0 ~ 100
  - `0`：墨盒空或查询失败（默认值）
  - `1-30`：墨量低，建议更换墨盒
  - `31-100`：墨量正常
- **更新机制**：
  - 当前版本：返回固定值 `0`（占位符）
  - 未来版本：将接入真实的墨量查询接口
- **查询失败**：如果墨量查询失败，返回 `0`

---

## 使用示例

### 1. 订阅话题（命令行）

```bash
# 查看话题信息
ros2 topic info /printer_status

# 实时查看消息内容
ros2 topic echo /printer_status

# 查看发布频率
ros2 topic hz /printer_status
```

### 2. 订阅话题（Python）

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class PrinterStatusSubscriber(Node):
    def __init__(self):
        super().__init__('printer_status_subscriber')
        self.subscription = self.create_subscription(
            String,
            'printer_status',
            self.status_callback,
            10
        )

    def status_callback(self, msg):
        # 解析 JSON
        status_data = json.loads(msg.data)

        # 访问各个喷码机的状态
        left_printer = status_data['printer_left']

        self.get_logger().info(
            f"左侧喷码机: "
            f"连接={left_printer['connected']}, "
            f"在线={left_printer['is_online']}, "
            f"状态={left_printer['status']}, "
            f"墨量={left_printer['ink_level']}%"
        )

        # 判断连接和在线状态
        if left_printer['connected']:
            self.get_logger().info("左侧喷码机已连接")
        elif left_printer['is_online']:
            self.get_logger().warning("左侧喷码机在线但未连接（可能是端口或防火墙问题）")
        else:
            self.get_logger().error("左侧喷码机离线（网络不通）")

        # 检查墨量告警
        if left_printer['ink_level'] < 30 and left_printer['ink_level'] > 0:
            self.get_logger().warning(
                f"左侧喷码机墨量低: {left_printer['ink_level']}%"
            )

def main():
    rclpy.init()
    node = PrinterStatusSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. 订阅话题（C++）

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"

class PrinterStatusSubscriber : public rclcpp::Node
{
public:
  PrinterStatusSubscriber() : Node("printer_status_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "printer_status", 10,
      std::bind(&PrinterStatusSubscriber::status_callback, this, std::placeholders::_1));
  }

private:
  void status_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // 解析 JSON
    auto status_data = nlohmann::json::parse(msg->data);

    // 访问左侧喷码机状态
    auto left_printer = status_data["printer_left"];
    bool connected = left_printer["connected"];
    bool is_online = left_printer["is_online"];
    std::string status = left_printer["status"];
    int ink_level = left_printer["ink_level"];

    RCLCPP_INFO(this->get_logger(),
                "左侧喷码机: 连接=%s, 在线=%s, 状态=%s, 墨量=%d%%",
                connected ? "是" : "否",
                is_online ? "是" : "否",
                status.c_str(), ink_level);

    // 判断连接和在线状态
    if (connected) {
      RCLCPP_INFO(this->get_logger(), "左侧喷码机已连接");
    } else if (is_online) {
      RCLCPP_WARN(this->get_logger(), "左侧喷码机在线但未连接（可能是端口或防火墙问题）");
    } else {
      RCLCPP_ERROR(this->get_logger(), "左侧喷码机离线（网络不通）");
    }

    // 墨量告警
    if (ink_level < 30 && ink_level > 0) {
      RCLCPP_WARN(this->get_logger(),
                  "左侧喷码机墨量低: %d%%", ink_level);
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrinterStatusSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

---

## 配置参数

### 修改发布频率

可以通过节点参数 `status_publish_rate` 调整发布频率：

```bash
# 设置为 5 Hz
ros2 run xline_inkjet_printer async_inkjet_printer_node \
  --ros-args -p status_publish_rate:=5.0

# 设置为 0.5 Hz（每2秒发布一次）
ros2 run xline_inkjet_printer async_inkjet_printer_node \
  --ros-args -p status_publish_rate:=0.5
```

**建议频率**：
- 实时监控：5-10 Hz
- 常规监控：1-2 Hz（默认）
- 低频监控：0.1-0.5 Hz

---

## 监控与告警

### 建议监控指标

1. **连接和在线状态监控**
   ```python
   if status['connected']:
       logger.info(f"{printer_name} 正常运行")
   elif status['is_online']:
       logger.warning(f"{printer_name} 在线但未连接（可能是端口或防火墙问题）")
   else:
       logger.error(f"{printer_name} 离线（网络不通或主机离线）")
   ```

2. **墨量告警**
   ```python
   if 0 < status['ink_level'] < 20:
       logger.error(f"{printer_name} 墨量严重不足: {status['ink_level']}%")
   elif 20 <= status['ink_level'] < 30:
       logger.warning(f"{printer_name} 墨量偏低: {status['ink_level']}%")
   ```

3. **综合诊断**
   ```python
   # 网络诊断
   if not status['connected'] and not status['is_online']:
       logger.critical(f"{printer_name} 网络不通，请检查网络连接和主机状态")
   elif not status['connected'] and status['is_online']:
       logger.error(f"{printer_name} 网络正常但 TCP 连接失败，请检查端口 {port} 和防火墙设置")
   ```

### 完整监控示例

```python
def monitor_printer_status(status_data):
    """监控喷码机状态并产生告警"""

    for printer_name, status in status_data.items():
        connected = status['connected']
        is_online = status['is_online']
        enabled = status['enabled']

        # 1. 连接和在线状态监控
        if connected:
            logger.debug(f"[{printer_name}] 正常运行")
        elif not enabled:
            logger.info(f"[{printer_name}] 已禁用")
        elif is_online:
            logger.error(f"[{printer_name}] 在线但未连接（检查端口或防火墙）")
        else:
            logger.critical(f"[{printer_name}] 离线（网络不通或主机离线）")

        # 2. 墨量监控
        ink_level = status['ink_level']
        if ink_level > 0:  # 忽略默认值0
            if ink_level < 10:
                logger.critical(f"[{printer_name}] 墨量极低: {ink_level}%，请立即更换！")
            elif ink_level < 20:
                logger.error(f"[{printer_name}] 墨量严重不足: {ink_level}%")
            elif ink_level < 30:
                logger.warning(f"[{printer_name}] 墨量偏低: {ink_level}%")

        # 3. 网络诊断
        if not connected and not is_online:
            logger.critical(
                f"[{printer_name}] 网络故障诊断："
                f"\n  - Ping 不可达"
                f"\n  - 建议检查：网络线缆、交换机、主机电源"
            )
        elif not connected and is_online:
            logger.error(
                f"[{printer_name}] TCP 连接故障诊断："
                f"\n  - Ping 可达（网络正常）"
                f"\n  - TCP 连接失败"
                f"\n  - 建议检查：端口号、防火墙设置、服务是否运行"
            )
```

---

## 与服务的对比

### `printer_status` 话题 vs 状态查询服务

| 特性 | `/printer_status` 话题 | `/printer_*/status` 服务 |
|------|------------------------|-------------------------|
| 调用方式 | 被动订阅 | 主动调用 |
| 更新频率 | 定时发布（1 Hz） | 按需查询 |
| 数据内容 | 基础状态信息 | 完整状态信息（含统计数据） |
| 实时性 | 准实时（最多延迟1秒） | 即时（查询时的状态） |
| 性能开销 | 低（一次发布，多个订阅） | 中（每次调用独立响应） |
| 适用场景 | 实时监控、界面显示 | 故障诊断、详细查询 |

**使用建议**：
- 需要**实时监控**多个喷码机 → 订阅 `printer_status` 话题
- 需要**详细统计信息** → 调用状态查询服务
- 需要**墨量信息** → 订阅 `printer_status` 话题（包含 `ink_level` 字段）

---

## 故障排除

### 1. 收不到话题消息

**问题**：`ros2 topic echo /printer_status` 无输出

**排查步骤**：
```bash
# 1. 检查节点是否运行
ros2 node list | grep inkjet_printer

# 2. 检查话题是否存在
ros2 topic list | grep printer_status

# 3. 检查话题发布者
ros2 topic info /printer_status

# 4. 查看节点日志
ros2 node info /inkjet_printer_node
```

### 2. 墨量始终为 0

**原因**：当前版本墨量查询接口尚未接入，返回默认值 0。

**解决方案**：等待后续版本接入真实墨量查询接口。

### 3. 数据解析错误

**问题**：JSON 解析失败

**解决方案**：
```python
import json

try:
    status_data = json.loads(msg.data)
except json.JSONDecodeError as e:
    logger.error(f"JSON 解析失败: {e}")
    logger.error(f"原始数据: {msg.data}")
```

### 4. 频率异常

**问题**：发布频率与预期不符

**检查方法**：
```bash
# 查看实际频率
ros2 topic hz /printer_status

# 查看参数配置
ros2 param get /inkjet_printer_node status_publish_rate
```

---

## 相关服务

与 `printer_status` 话题配合使用的服务：

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/printer_left/status` | `std_srvs/Trigger` | 查询左侧喷码机详细状态 |
| `/printer_center/status` | `std_srvs/Trigger` | 查询中间喷码机详细状态 |
| `/printer_right/status` | `std_srvs/Trigger` | 查询右侧喷码机详细状态 |
| `/printer/quick_command` | `xline_msgs/QuickCommand` | 发送快捷命令（含墨量查询） |

**墨量查询服务**：
```bash
# 查询左侧喷码机墨量（通过快捷命令服务）
ros2 service call /printer/quick_command xline_msgs/srv/QuickCommand \
  "{action: 'ink_level', printer_name: 'left', param: 0}"
```

---

## 更新历史

| 版本 | 日期 | 变更内容 |
|------|------|---------|
| v3.0.0 | 2025-11-05 | **重大更新**：新增 `is_online` 字段（IP 在线检测），恢复 `status` 字段，移除 `status_code` 和 `status_message` |
| v2.0.0 | 2025-11-05 | 将 `status` 字符串字段替换为 `status_code`（整数）和 `status_message`（字符串） |
| v1.2.0 | 2025-11-05 | 新增 `ink_level` 字段（墨量余量） |
| v1.1.0 | 2025-10-20 | 新增 `auto_connect` 和 `enabled` 字段 |
| v1.0.0 | 2025-10-15 | 初始版本，包含基础状态信息 |

**⚠️ 不兼容变更（v3.0.0）**：
- 新增 `is_online` 字段：IP 在线状态（Ping 检测）
- 恢复 `status` 字段：人类可读的状态描述
- 移除 `status_code` 和 `status_message` 字段

**Ping 检测机制（v3.0.0 新增）**：
- TCP 连接时：不执行 Ping 检测（避免浪费资源）
- TCP 未连接时：每 2 秒执行一次 Ping 检测
- 用于网络故障诊断

---

## 参考文档

- [README.md](./README.md) - 完整的包文档
- [ROS 2 话题文档](https://docs.ros.org/en/humble/Tutorials/Topics/Understanding-ROS2-Topics.html)
- 喷码机协议规范：请联系维护者获取

---

## 联系方式

如有问题或建议，请联系：

- **维护者**：xline
- **邮箱**：maintainer@example.com
- **项目路径**：`/root/xline_ws/xline_base_controller/src/xline_inkjet_printer`

---

**最后更新**：2025-11-05
