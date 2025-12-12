# RPP Follow Controller - 策略模式重构版

## 概述

本项目是对原有 `RPPController` 的策略模式重构。通过将圆形路径和曲线路径的特有逻辑分离到独立的策略类中，提高了代码的可维护性和可扩展性。

## 架构设计

### 类图

```
                    ┌─────────────────────┐
                    │   PathStrategy      │ (抽象基类)
                    │   <<interface>>     │
                    ├─────────────────────┤
                    │ + getTypeName()     │
                    │ + setPlan()         │
                    │ + isGoalReached()   │
                    │ + computeAngularVel │
                    │ + reset()           │
                    └──────────┬──────────┘
                               │
              ┌────────────────┴────────────────┐
              │                                 │
              ▼                                 ▼
┌─────────────────────────┐     ┌─────────────────────────┐
│   CurvePathStrategy     │     │   CirclePathStrategy    │
├─────────────────────────┤     ├─────────────────────────┤
│ - 基于距离的目标判定     │     │ - 基于累计角度的目标判定 │
│ - 标准 Pure Pursuit     │     │ - 基准角速度偏差约束     │
│ - 支持后退模式          │     │ - 航向预对准             │
│                         │     │ - 根据半径调整速度       │
└─────────────────────────┘     └─────────────────────────┘

              ┌─────────────────────────────────┐
              │          RPPController          │
              ├─────────────────────────────────┤
              │ - 共享的 Pure Pursuit 核心      │
              │ - 滤波器系统                    │
              │ - 路径处理                      │
              │ - 栅格图可视化                  │
              │ + path_strategy_: PathStrategy* │ (组合关系)
              └─────────────────────────────────┘
```

### 文件结构

```
rpp_refactored/
├── include/xline_follow_controller/
│   ├── path_strategy.hpp           # 策略基类接口
│   ├── curve_path_strategy.hpp     # 曲线路径策略
│   ├── circle_path_strategy.hpp    # 圆形路径策略
│   ├── rpp_follow_controller.hpp   # 重构后的控制器
│   ├── base_follow_controller.hpp  # 基类（原有）
│   ├── follow_common.hpp           # 公共工具类（原有）
│   ├── yaml_parser.hpp             # YAML解析器（原有）
│   ├── path_utils.hpp              # 路径工具（原有）
│   └── logging_compat.hpp          # 日志兼容（原有）
├── src/
│   ├── path_strategy_factory.cpp   # 策略工厂
│   ├── curve_path_strategy.cpp     # 曲线路径策略实现
│   ├── circle_path_strategy.cpp    # 圆形路径策略实现
│   └── rpp_follow_controller.cpp   # 重构后的控制器实现
├── config/
│   ├── rpp_circle.yaml             # 圆形路径配置
│   ├── rpp_curve.yaml              # 曲线路径配置
│   └── line.yaml                   # 直线路径配置
└── README.md                       # 本文档
```

## 关键差异点

| 维度 | CurvePathStrategy | CirclePathStrategy |
|------|-------------------|-------------------|
| **目标判定** | 距离判定 `dist < goal_dist_tol` | 累计角度判定 `acc_angle >= total_angle` |
| **角速度计算** | 标准 PP: `v × curvature` | 基准角速度 + 偏差约束 |
| **速度调整** | 使用配置参数 | 根据半径动态调整 |
| **预对准** | 可选（后退模式） | 必需（切入圆弧前） |
| **配置文件** | `rpp_curve.yaml` | `rpp_circle.yaml` |

## 使用方法

### 曲线路径跟随

```cpp
// 创建控制器（默认使用曲线路径策略）
auto controller = std::make_shared<RPPController>();
controller->initialize();

// 设置路径
nav_msgs::msg::Path path = ...;
controller->setPlan(path);

// 计算速度命令
geometry_msgs::msg::TwistStamped cmd_vel;
controller->computeVelocityCommands(current_pose, current_velocity, cmd_vel);
```

### 圆形路径跟随

```cpp
// 创建控制器
auto controller = std::make_shared<RPPController>();
controller->initialize();

// 设置角度范围
controller->setAngleRange(0.0, M_PI);  // 半圆

// 设置圆形路径
controller->setPlanForCircle(center_x, center_y, radius, robot_pose);

// 计算速度命令
geometry_msgs::msg::TwistStamped cmd_vel;
controller->computeVelocityCommands(current_pose, current_velocity, cmd_vel);
```

### 后退模式

```cpp
controller->setBackFollow(true);
controller->setPlan(path);
```

## 扩展新策略

如需添加新的路径类型（如椭圆路径），只需：

1. 创建新的策略类继承 `PathStrategy`：

```cpp
class EllipsePathStrategy : public PathStrategy {
public:
    std::string getTypeName() const override { return "ellipse"; }
    bool setPlan(const nav_msgs::msg::Path& path) override { ... }
    bool isGoalReached(const PathStrategyContext& ctx) override { ... }
    void computeAngularVelocity(const PathStrategyContext& ctx,
                                 PathStrategyResult& result) override { ... }
    void reset() override { ... }
    void updateParameters(const std::string& config_path) override { ... }
};
```

2. 在工厂函数中注册：

```cpp
PathStrategy::UniquePtr createPathStrategy(PathStrategyType type) {
    switch (type) {
        case PathStrategyType::ELLIPSE:
            return std::make_unique<EllipsePathStrategy>();
        // ...
    }
}
```

3. 在 `RPPController` 中添加对应的设置接口。

## 优势

1. **单一职责**：每个策略类只负责一种路径类型的特有逻辑
2. **开闭原则**：添加新路径类型无需修改现有代码
3. **易于测试**：策略可以独立进行单元测试
4. **代码复用**：共享逻辑（滤波、路径处理等）集中在控制器中
5. **运行时切换**：可以在运行时动态切换策略

## 编译

确保已安装以下依赖：
- ROS2 (推荐 Humble 或更新版本)
- OpenCV
- Eigen3
- yaml-cpp
- tf2_geometry_msgs

**重要提示**: 由于系统限制，`package.xml` 文件中的包名标签可能显示不正确。请在编译前手动检查并确保第4行为：
```xml
  <name>xline_follow_controller</name>
```

在 CMakeLists.txt 中添加新的源文件并编译即可。

## 注意事项

1. 策略切换时会自动重置控制器状态
2. 圆形路径不支持后退模式
3. 参数更新会同时更新控制器和当前策略的参数
4. 日志输出通过 ROS2 日志系统，策略类共享控制器的 logger
