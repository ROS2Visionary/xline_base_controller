# xline_localization

状态估计节点,融合 IMU 和激光反射板位置数据,估计机器人实时位姿。

## 功能特性

- **传感器融合**: 融合 IMU 姿态和激光反射板位置
- **姿态计算**: 基于 IMU 姿态差值计算机器人朝向
- **位置计算**: 基于反射板位置和偏移量计算机器人基座位置
- **姿态校准服务**: 提供服务接口进行初始姿态校准

## 参数配置

参数通过 YAML 文件配置,包内路径: `src/xline_localization/config/localization_params.yaml`

```yaml
localization:
  ros__parameters:
    reflector_to_base_x: 0.0  # 反射板到基座 X 偏移(米)
    reflector_to_base_y: 0.0  # 反射板到基座 Y 偏移(米)
    reflector_to_base_z: 0.0  # 反射板到基座 Z 偏移(米)
```

**注意**:
- 这些参数是必需的,必须在 YAML 文件中设置,没有默认值
- 配置文件会自动安装到 `install/xline_localization/share/xline_localization/config/`
- 节点启动时会自动从包的共享目录加载配置

## 话题

### 订阅
- `/imu` (sensor_msgs/Imu): IMU 数据
- `/reflector_position` (geometry_msgs/PointStamped): 激光反射板位置

### 发布
- `/estimated_pose` (geometry_msgs/PoseStamped): 估计的机器人位姿(每次接收到反射板位置时发布)

## 服务

- `~/calibrate_pose` (std_srvs/Trigger): 姿态校准服务
  - 调用此服务记录当前 IMU 和反射板数据作为初始姿态基准
  - 必须在节点启动后调用一次,才能开始发布位姿

## 使用方法

### 1. 启动节点

**方法1**: 使用包内默认配置(推荐)
```bash
ros2 run xline_localization localization
```
节点会自动从包的安装目录加载 `config/localization_params.yaml`

**方法2**: 指定自定义配置文件
```bash
ros2 run xline_localization localization --ros-args -p config_file:=/path/to/your/config.yaml
```

### 2. 进行姿态校准

节点启动后,必须调用校准服务:

```bash
ros2 service call /localization/calibrate_pose std_srvs/srv/Trigger
```

### 3. 查看估计位姿

```bash
ros2 topic echo /estimated_pose
```

## 工作原理

### 位置计算

参考 `daosnrs_localization.cpp` 的实现:

1. **反射板位姿**: 从激光雷达获取反射板位置,从 IMU 计算姿态
2. **坐标转换**: `global_to_base = global_to_reflector * reflector_to_base_tf`
3. **输出**: 机器人基座在全局坐标系中的位姿

### 姿态计算

使用 IMU 姿态差值方法:

```cpp
robot_yaw = (imu_current_yaw - imu_initial_yaw) + robot_initial_yaw
```

- `imu_initial_yaw`: 校准时记录的 IMU 初始航向
- `robot_initial_yaw`: 校准时记录的机器人初始航向
- `imu_current_yaw`: 当前 IMU 航向

### 初始化流程

1. 节点启动,加载 YAML 配置
2. 等待传感器数据
3. 用户调用 `~/calibrate_pose` 服务
4. 记录 IMU 和反射板初始状态
5. 开始发布融合位姿

## 依赖项

- rclcpp
- sensor_msgs
- geometry_msgs
- std_srvs
- tf2
- tf2_geometry_msgs
- Eigen3
- yaml-cpp

## 坐标系

- **map**: 全局坐标系
- **base_link**: 机器人基座坐标系
- **reflector**: 反射板坐标系

坐标转换关系: `map -> reflector -> base_link`