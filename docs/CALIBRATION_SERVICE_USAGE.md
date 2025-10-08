# 姿态校正服务使用说明

## 概述

`motion_control_center` 节点提供了姿态校正服务，允许其他节点调用该服务来执行完整的姿态校正流程。该服务会自动控制底盘移动并调用定位节点完成姿态校准。

## 服务信息

- **服务名**: `/motion_control/execute_calibration`
- **服务类型**: `std_srvs/srv/Trigger`
- **功能**: 执行姿态校正（控制底盘前进 + 调用定位校准）

### 请求参数

无需参数（使用默认参数）

### 响应字段

- `success` (bool): 校正是否成功
- `message` (string): 详细信息

### 默认校准参数

- 前进速度: 0.05 m/s
- 持续时间: 3.0 秒

## 使用方式

### 方式1: 命令行调用

最简单的方式，适合快速测试：

```bash
# 调用姿态校正服务
ros2 service call /motion_control/execute_calibration std_srvs/srv/Trigger
```

### 方式2: Python脚本调用

使用提供的示例脚本：

```bash
# 进入脚本目录
cd /root/xline_ws/xline_base_controller/src/xline_base_controller/scripts

# 运行示例（同步方式）
python3 calibration_client_example.py

# 或者直接执行
./calibration_client_example.py
```

**Python代码示例**：

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# 创建节点
rclpy.init()
node = rclpy.create_node('my_calibration_client')

# 创建服务客户端
client = node.create_client(Trigger, '/motion_control/execute_calibration')

# 等待服务可用
if client.wait_for_service(timeout_sec=10.0):
    # 创建请求
    request = Trigger.Request()

    # 调用服务
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30)

    # 获取结果
    if future.done():
        response = future.result()
        if response.success:
            print(f"✅ 校正成功: {response.message}")
        else:
            print(f"❌ 校正失败: {response.message}")
else:
    print("❌ 服务不可用")

rclpy.shutdown()
```

### 方式3: C++ 代码调用

编译并运行C++示例：

```bash
# 编译（如果已添加到CMakeLists.txt）
cd /root/xline_ws/xline_base_controller
colcon build --packages-select xline_base_controller

# 运行示例
ros2 run xline_base_controller calibration_client_example
```

**C++代码示例**：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("my_calibration_client");

    // 创建服务客户端
    auto client = node->create_client<std_srvs::srv::Trigger>(
        "/motion_control/execute_calibration");

    // 等待服务可用
    if (client->wait_for_service(10s)) {
        // 创建请求
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        // 调用服务
        auto future = client->async_send_request(request);

        // 等待响应（30秒超时）
        if (future.wait_for(30s) == std::future_status::ready) {
            auto response = future.get();

            if (response->success) {
                RCLCPP_INFO(node->get_logger(), "✅ 校正成功: %s",
                           response->message.c_str());
            } else {
                RCLCPP_ERROR(node->get_logger(), "❌ 校正失败: %s",
                            response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "❌ 服务调用超时");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "❌ 服务不可用");
    }

    rclcpp::shutdown();
    return 0;
}
```

## 注意事项

### 调用前提条件

1. ✅ `motion_control_center` 节点必须已启动
2. ✅ `localization` 节点必须已启动
3. ✅ **当前没有任务正在执行**（否则会被拒绝）

### 返回状态

| 情况 | success | message |
|------|---------|---------|
| 成功 | `true` | "姿态校正成功完成" |
| 有任务执行中 | `false` | "拒绝校正：当前有任务正在执行中，请先完成或取消当前任务" |
| 校准失败 | `false` | "姿态校正失败，请查看日志了解详情" |

### 执行流程

当调用服务后，系统会自动执行以下步骤：

1. 检查是否有任务正在执行
2. 等待定位服务可用
3. 异步调用定位节点的校准服务 `/localization/calibrate_pose`
4. 控制底盘以0.05m/s速度前进3秒
5. 停止底盘移动
6. 等待定位服务返回结果
7. 返回校正结果给调用者

### 超时设置建议

- 服务等待: 10秒（等待服务可用）
- 响应等待: 30秒（等待校正完成）

## 故障排查

### 问题1: 服务不可用

```
❌ 服务不可用，请确保 motion_control_center 节点已启动
```

**解决方法**：
```bash
# 检查节点是否运行
ros2 node list | grep motion_control_center

# 检查服务是否存在
ros2 service list | grep execute_calibration

# 启动节点
ros2 run xline_base_controller motion_control_center
```

### 问题2: 任务执行中被拒绝

```
❌ 拒绝校正：当前有任务正在执行中
```

**解决方法**：
- 等待当前任务完成
- 或使用取消服务取消当前任务

### 问题3: 定位服务超时

```
❌ 校准服务不可用，请确保 localization 节点已启动
```

**解决方法**：
```bash
# 检查定位节点
ros2 node list | grep localization

# 启动定位节点
ros2 run xline_localization localization
```

## 与其他服务的关系

```
调用者 (其他节点)
    |
    | 调用服务
    v
motion_control_center (/motion_control/execute_calibration)
    |
    |-- 控制底盘移动 (/cmd_vel)
    |
    |-- 调用定位校准 (/localization/calibrate_pose)
    |
    v
localization 节点
```

## 完整使用示例

假设您有一个任务调度节点，需要在执行任务前先进行姿态校正：

```python
# 任务调度节点示例
class TaskScheduler(Node):
    def __init__(self):
        super().__init__('task_scheduler')

        # 创建校准服务客户端
        self.calibration_client = self.create_client(
            Trigger, '/motion_control/execute_calibration')

    def execute_task(self):
        # 1. 先执行姿态校正
        if not self.calibrate_pose():
            self.get_logger().error("姿态校正失败，取消任务")
            return False

        # 2. 执行实际任务
        self.get_logger().info("开始执行任务...")
        # ... 任务逻辑 ...

        return True

    def calibrate_pose(self):
        """执行姿态校正"""
        request = Trigger.Request()
        future = self.calibration_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30)

        if future.done():
            response = future.result()
            return response.success
        return False
```

## 参考文件

- 服务实现: `src/xline_base_controller/src/motion_control_center.cpp:647`
- C++ 示例: `src/xline_base_controller/src/calibration_client_example.cpp`
- Python 示例: `src/xline_base_controller/scripts/calibration_client_example.py`
- 服务声明: `src/xline_base_controller/include/xline_base_controller/motion_control_center.hpp:137`
