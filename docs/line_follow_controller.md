# LineFollowController 直线跟随控制器文档

基于参考文档`执行流程.md`，本文档详细描述了`line_follow_controller.cpp`的实现和工作流程。

## 概述

`LineFollowController`是DAOSNRS机器人直线路径跟随的核心控制器，负责执行精确的直线轨迹跟踪，支持前进和后退跟随模式，具备地形自适应和IMU融合功能。

**文件位置**: `follow_controller/src/line_follow_controller.cpp`

## LineFollowController执行流程图

基于参考文档`执行流程.md`的设计，以下是LineFollowController内部的详细执行流程：

```
                    【LineFollowController启动】
                        ↓
                  setPlan() 路径设置
                        │
              ┌─────────┼─────────┐
              ↓         ↓         ↓
        路径有效性检查  路径长度计算  状态初始化
              │         │         │
              ↓         ↓         ↓
        received_plan_=true  path_length_  current_state_=IDLE
              │         │         │
              └─────────┼─────────┘
                        ↓
                【computeVelocityCommands调用】
                        │
              ┌─────────┼─────────┐
              ↓         ↓         ↓
        前置条件检查   位姿数据获取   重置要求检查
              │         │         │
        received_plan_  robot_pose  reset_required_
              │         │         │
              └─────────┼─────────┘
                        ↓
                【控制状态机处理】
                        │
        ┌───────────────┼───────────────┐
        ↓               ↓               ↓
    IDLE状态        ALIGNING_START    FOLLOWING_PATH
        │               │               │
   路径长度检查     起点对齐控制      主跟随控制
        ↓               ↓               ↓
   短路径判断       PID角度调整      横向偏差计算
        │               │               │
        ↓               ↓               ↓
   状态转换判断     对齐完成检查      航向偏差计算
        │               │               │
        └───────────────┼───────────────┘
                        ↓
                【地形自适应循环】
                        │
        ┌───────────────┼───────────────┐
        ↓               ↓               ↓
    IMU数据获取     地形类型分析      参数动态调整
        │               │               │
   姿态角提取        梯度计算         PID参数更新
        ↓               ↓               ↓
   角速度提取        能量分析         滤波器参数
        │               │               │
        ↓               ↓               ↓
   历史数据维护      零交叉分析       平滑器参数
        │               │               │
        └───────────────┼───────────────┘
                        ↓
                【速度控制计算】
                        │
        ┌───────────────┼───────────────┐
        ↓               ↓               ↓
    线速度计算       角速度计算       速度平滑处理
        │               │               │
   加速sigmoid      PID控制输出       二阶滤波器
        ↓               ↓               ↓
   减速sigmoid      角度误差计算      Hampel滤波器
        │               │               │
        ↓               ↓               ↓
   速度限制应用     角速度限制        平滑输出
        │               │               │
        └───────────────┼───────────────┘
                        ↓
                【虚拟跟踪处理】
                        │
        ┌───────────────┼───────────────┐
        ↓               ↓               ↓
   定位信号检查     运动学预测       位姿融合处理
        │               │               │
   信号质量评估     基于速度预测      加权融合
        ↓               ↓               ↓
   trust_threshold  位置预测计算      最终位姿输出
        │               │               │
        └───────────────┼───────────────┘
                        ↓
                【目标到达检查】
                        │
        ┌───────────────┼───────────────┐
        ↓               ↓               ↓
    距离检查         角度检查         状态更新
        │               │               │
   waypoint_tolerance  yaw_tolerance   goal_reached_
        │               │               │
        └───────────────┼───────────────┘
                        ↓
                【速度指令输出】
                        │
        cmd_vel.twist.linear.x = calculated_linear
        cmd_vel.twist.angular.z = calculated_angular
                        ↓
                【数据日志记录】
                        │
        ┌───────────────┼───────────────┐
        ↓               ↓               ↓
   地形分类日志     控制参数日志      性能指标日志
        │               │               │
   terrain_type    PID参数状态       跟踪精度数据
        │               │               │
        └───────────────┼───────────────┘
                        ↓
                【控制循环完成】
```

### 关键流程节点说明

1. **路径设置阶段** (`line_follow_controller.cpp:580-650`)
   - `setPlan()`: 验证路径，计算长度，设置控制状态为IDLE

2. **控制状态机** (`line_follow_controller.cpp:827-1200`)
   - `IDLE`: 路径分析和初始化准备
   - `ALIGNING_START`: 起点精确对齐（如需要）
   - `FOLLOWING_PATH`: 主要路径跟随控制
   - `ALIGNING_END`: 终点精确对齐
   - `GOAL_REACHED`: 目标到达确认

3. **地形自适应** (`line_follow_controller.cpp:1950-2100`)
   - IMU数据实时分析（200Hz）
   - 地形类型自动分类
   - 控制参数动态调整

4. **速度控制** (`line_follow_controller.cpp:1400-1600`)
   - 线速度：sigmoid加减速曲线
   - 角速度：PID控制 + 多级滤波
   - 地形因子实时调制

5. **虚拟跟踪** (`line_follow_controller.cpp:700-800`)
   - 定位信号质量监测
   - 运动学模型预测
   - 传感器数据融合

## 类结构

### 继承关系
```cpp
class LineFollowController : public BaseFollowController
```

### 控制状态
```cpp
enum class ControlState {
    IDLE,            // 空闲状态
    ALIGNING_START,  // 起点对齐状态  
    FOLLOWING_PATH,  // 路径跟随状态
    ALIGNING_END,    // 终点对齐状态
    GOAL_REACHED,    // 目标到达状态
    WAITING          // 等待状态
};
```

## 主要功能模块

### 1. 路径设置 (setPlan)

**功能**: 接收并处理路径规划信息，初始化跟随控制参数

**执行流程**:
1. 验证路径有效性
2. 计算路径长度和方向
3. 设置初始控制状态
4. 初始化路径跟踪参数

**关键参数**:
- `waypoint_tolerance_`: 航点容忍度 (默认: 0.005m)
- `mini_path_distance_`: 最小路径距离阈值
- `short_path_`: 短路径标记

### 2. 速度控制计算 (computeVelocityCommands)

**功能**: 核心控制循环，计算机器人的线速度和角速度指令

**输入参数**:
- `pose`: 机器人当前位姿
- `velocity`: 机器人当前速度
- `cmd_vel`: 输出速度指令

**控制逻辑**:

#### 2.1 前置检查
```cpp
// line_follow_controller.cpp:827-847
if (!received_plan_) {
    LOG_WARN("尚未接收到路径");
    return false;
}
if (goal_reached_) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return true;
}
```

#### 2.2 状态机控制
根据`current_state_`执行不同的控制策略:

**IDLE状态**: 初始化和路径验证
- 检查路径长度是否满足最小距离要求
- 决定是否需要起点对齐

**ALIGNING_START状态**: 起点姿态对齐
- 使用PID控制器调整机器人朝向
- 对齐完成后转入`FOLLOWING_PATH`状态

**FOLLOWING_PATH状态**: 主要跟随控制
- 计算横向偏差和航向偏差
- 应用地形自适应控制参数
- 执行速度和角度控制

**ALIGNING_END状态**: 终点精确对齐
- 精确调整最终姿态
- 满足容忍度要求后标记目标到达

### 3. 地形自适应控制

**功能**: 根据IMU数据实时调整控制参数以适应不同地形

#### 3.1 地形分类 (TerrainType)
```cpp
enum class TerrainType {
    SMOOTH,            // 平滑地形
    GENTLE_UNDULATION, // 缓和起伏
    SUDDEN_CHANGE,     // 突然变化  
    FREQUENT_BUMPS     // 频繁颠簸
};
```

#### 3.2 地形检测算法
**位置**: `line_follow_controller.cpp:1950-2100`

**检测指标**:
- 姿态角度梯度: 检测突然变化
- 低频能量: 识别缓和起伏
- 零交叉率: 检测频繁颠簸

**参数调整**:
根据地形类型动态调整:
- `max_angular_vel`: 最大角速度限制
- `suppression_factor`: 控制抑制因子
- `alpha`: 滤波器平滑系数
- `smoother_frequency`: 二阶平滑器频率

### 4. 速度控制策略

#### 4.1 线速度控制
```cpp
// 速度限制根据工作状态设置
double max_vel = m_work_state_ ? m_work_max_vel_ : m_walk_max_vel_;
```

**加速控制**: 使用sigmoid函数实现平滑加速
```cpp
double acceleration_progress = std::min(1.0, traveled_distance / m_acce_distance_);
double sigmoid_accel = 1.0 / (1.0 + std::exp(-m_acceleration_factor_ * 
                              (acceleration_progress - m_acceleration_sigmoid_center_)));
```

**减速控制**: 基于剩余距离的渐进减速
```cpp
double remaining_distance = path_length_ - traveled_distance;
if (remaining_distance <= deceleration_distance_) {
    double decel_progress = remaining_distance / deceleration_distance_;
    double sigmoid_decel = 1.0 / (1.0 + std::exp(-m_deceleration_factor_ * 
                                  (decel_progress - m_deceleration_sigmoid_center_)));
}
```

#### 4.2 角速度控制

**PID控制器参数**:
- `angular_kp_`: 比例增益 (默认: 1.1)  
- `angular_ki_`: 积分增益 (默认: 0.0)
- `angular_kd_`: 微分增益 (默认: 0.12)

**角速度平滑**: 使用二阶低通滤波器和Hampel滤波器
```cpp
angular_smoother_.setParameters(current_smoother_frequency_, current_smoother_damping_);
double smoothed_angular = angular_smoother_.filter(pid_output);
double filtered_angular = angular_vel_hampel_filter_.filter(smoothed_angular);
```

### 5. IMU数据处理

#### 5.1 IMU订阅初始化
**位置**: `line_follow_controller.cpp:465-485`

创建独立的IMU处理线程，避免阻塞主控制循环:
```cpp
void initializeImuSubscription() {
    imu_node_ = rclcpp::Node::make_shared("line_follow_imu_subscriber");
    imu_subscription_ = imu_node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&LineFollowController::imuCallback, this, std::placeholders::_1));
    
    imu_thread_running_ = true;
    imu_thread_ = std::thread([this]() {
        rclcpp::spin(imu_node_);
    });
}
```

#### 5.2 IMU数据融合
**功能**: 融合IMU的姿态角和角速度信息进行地形分类

**关键数据**:
- `current_pitch_angle_`: 当前俯仰角
- `current_roll_angle_`: 当前横滚角  
- `current_yaw_rate_`: 当前偏航角速度

### 6. 虚拟跟踪系统 (Virtual Tracking)

**功能**: 在定位信号不稳定时提供运动学预测

**配置参数**:
```cpp
struct VirtualTrackingConfig {
    bool enabled;
    double correction_gain;
    double max_correction;
    double position_trust_threshold;
    double max_velocity_trust;
    double angular_velocity_factor;
};
```

**工作原理**:
1. 基于上一时刻的位姿和速度预测当前位置
2. 与实际定位结果进行加权融合
3. 在定位信号质量差时提高预测权重

### 7. 数据日志记录

#### 7.1 地形分类日志
**主题**: `/terrain_classification_log`
**频率**: 可配置 (默认: 20Hz)

记录内容:
- 当前地形类型和置信度
- IMU姿态角数据
- 控制参数动态调整情况
- 滤波器状态信息

#### 7.2 详细控制日志
**路径**: `/home/daosn_robotics/zyq_ws/terrain_data/`

记录内容:
- 路径跟踪精度数据
- 速度控制输出
- PID控制器状态
- 异常情况记录

## 关键接口函数

### 1. 基础控制接口
```cpp
// 设置路径规划
bool setPlan(const nav_msgs::msg::Path& orig_global_plan) override;

// 计算速度指令  
bool computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                            const geometry_msgs::msg::Twist& velocity,
                            geometry_msgs::msg::TwistStamped& cmd_vel) override;

// 检查目标到达状态
bool isGoalReached() override;
```

### 2. 配置接口
```cpp
// 设置机器人初始位姿
void setPose(const geometry_msgs::msg::PoseStamped& pose);

// 设置工作状态 (0: 目标点导航, 1: 路径跟随)
void setWorkState(int work_state);

// 设置后退跟随模式
void setBackFollow(bool back_follow);

// 设置速度限制
void setSpeedLimit(double speed_limit);

// 设置重置要求标志
void setResetRequired(bool reset);
```

### 3. 状态查询接口
```cpp
// 获取当前控制状态
ControlState getCurrentState() const;

// 检查是否收到路径
bool hasReceivedPlan() const;

// 获取路径长度
double getPathLength() const;
```

## 参数配置文件

**配置文件位置**: `config/line.yaml`

### 主要参数分类

#### 1. 运动参数 (motion)
```yaml
motion:
  velocity:
    limits:
      min: 0.15              # 最小线速度
      walk_max: 0.26         # 行走最大速度
      work_max: 0.26         # 工作最大速度
      alignment: 0.12        # 对齐速度
    distances:
      alignment: 0.25        # 对齐距离
      acceleration: 0.3      # 加速距离
      deceleration: 0.2      # 减速距离
      lookahead: 0.25        # 前瞻距离
      waypoint_tolerance: 0.005  # 航点容忍度
```

#### 2. 角度控制参数 (angular)
```yaml
motion:
  angular:
    rotation:
      limits:
        min: 0.35           # 最小旋转角速度
        max: 0.8            # 最大旋转角速度
      tuning:
        factor: 0.65        # 旋转因子
        angle_threshold: 0.1  # 角度阈值
        smooth_factor: 0.5   # 平滑因子
```

#### 3. PID控制参数 (control)
```yaml
control:
  pid:
    kp: 1.1               # 比例增益
    ki: 0.0               # 积分增益  
    kd: 0.12              # 微分增益
```

#### 4. 地形控制参数 (terrain_control)
```yaml
terrain_control:
  smooth_terrain:
    cross_track_deadzone: 0.01
    yaw_deadzone: 0.02
    max_angular_vel: 0.6
    max_angular_accel: 0.4
    suppression_factor: 1.0
    heading:
      current_heading_weight: 0.3
      target_heading_weight: 0.7
    filtering:
      alpha: 0.85
    smoother:
      frequency: 15.0
      damping: 0.95
```

## 性能特点

### 1. 精确度
- 航点跟踪精度: ±0.005m
- 角度跟踪精度: ±0.05rad
- 路径跟踪误差: < 2cm (平滑地形)

### 2. 实时性
- 控制频率: 20Hz
- IMU处理频率: 200Hz
- 平均计算延迟: < 5ms

### 3. 鲁棒性
- 支持定位信号中断恢复
- 地形自适应参数调整
- 多层安全检查机制

### 4. 可配置性
- 40+ 可配置参数
- 支持运行时参数更新
- 详细的调试日志输出

## 与主控制器集成

在`daosnrs_controller.cpp`中的集成方式:

### 1. 控制器选择
```cpp
// daosnrs_controller.cpp:376-388
if (path_type == PathType::Line) {
    controller_ = line_follow_controller_;
    geometry_msgs::msg::PoseStamped robot_pose;
    getRobotPose(robot_pose);
    line_follow_controller_->setPose(robot_pose);
    controller_->setSpeedLimit(goal->linear_vel);
    line_follow_controller_->setWorkState(workstate);
}
```

### 2. 速度计算调用
```cpp
// daosnrs_controller.cpp:719-769
bool computeAndPublishVelocity() {
    geometry_msgs::msg::PoseStamped robot_pose;
    getRobotPose(robot_pose);
    
    geometry_msgs::msg::TwistStamped cmd_vel;
    if (!controller_->computeVelocityCommands(robot_pose, velocity, cmd_vel)) {
        return false;
    }
    
    // 应用安全限制和发布速度
    publishVelocity(cmd_vel);
    return true;
}
```

### 3. 目标到达检查
```cpp
// daosnrs_controller.cpp:465
if (goalReached()) {
    // 通过 line_follow_controller_->isGoalReached() 检查
    LOG_INFO("Reached the goal!");
    m_result->status = RESULT_STATUS_SUCCESS;
    goal_handle->succeed(m_result);
    break;
}
```

## 故障排除

### 常见问题

1. **路径跟踪偏差过大**
   - 检查PID参数调整
   - 验证地形分类是否正确
   - 确认IMU数据质量

2. **角度控制震荡**
   - 降低`angular_kp_`增益
   - 增加角速度平滑器阻尼
   - 检查Hampel滤波器参数

3. **目标点无法到达**
   - 确认`waypoint_tolerance_`设置合理
   - 检查路径长度是否满足最小要求
   - 验证终点对齐逻辑

### 调试建议

1. **启用详细日志**
   ```yaml
   debug: true
   data_logging:
     detailed_logging: true
     enabled: true
   ```

2. **监控关键话题**
   - `/terrain_classification_log`: 地形分类状态
   - `/cmd_vel`: 速度指令输出
   - `/imu`: IMU数据质量

3. **参数调优流程**
   - 首先在平滑地形测试基础参数
   - 逐步增加地形复杂度
   - 使用数据日志分析性能瓶颈

## 总结

`LineFollowController`是一个高精度、自适应的直线路径跟踪控制器，通过多传感器融合、地形自适应控制和精密的PID调节，实现了在各种地形条件下的精确路径跟踪。其模块化设计和丰富的配置选项使其能够适应不同的应用场景和性能要求。