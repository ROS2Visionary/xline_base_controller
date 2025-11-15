#include "xline_obstacle_detector/obstacle_detector_node.hpp"
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace xline_obstacle_detector
{

ObstacleDetectorNode::ObstacleDetectorNode()
  : Node("xline_obstacle_detector_node"), first_scan_received_(false)
{
  // 获取参数
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("xline_obstacle_detector");
  std::string config_file_path = package_share_directory + "/config/obstacle_detector_config.yaml";

  xline::YamlParser::YamlParser parser(config_file_path);

  // 读取机器人几何参数
  robot_length_ = parser.getParameter<double>("robot_geometry.length");
  robot_width_ = parser.getParameter<double>("robot_geometry.width");
  
  // 读取激光雷达位置参数
  lidar_x_ = parser.getParameter<double>("lidar_position.x");
  lidar_y_ = parser.getParameter<double>("lidar_position.y");

  front_detection_distance_ = parser.getParameter<double>("detection_distances.front");
  back_detection_distance_ = parser.getParameter<double>("detection_distances.back");
  left_detection_distance_ = parser.getParameter<double>("detection_distances.left");
  right_detection_distance_ = parser.getParameter<double>("detection_distances.right");

  // 注意：新算法不再需要读取角度范围参数

  laser_topic_ = parser.getParameter<std::string>("topics.laser_topic");

  mask_angle_ = parser.getParameter<double>("mask.mask_angle");
  mask_range_ = parser.getParameter<double>("mask.mask_range");



  // 初始化屏蔽区域
  initializeMaskZones();

  // 创建订阅者
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleDetectorNode::laserScanCallback, this, std::placeholders::_1));

  // 创建发布者
  // 主要发布者：使用Float32MultiArray发布障碍物距离数组
  obstacle_detected_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/obstacle_detected", 10);

  // 状态发布者
  obstacle_status_pub_ = this->create_publisher<std_msgs::msg::String>("/status", 10);
  
  // 新增：简单Bool状态发布者
  obstacle_bool_pub_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_detected_bool", 10);


  // 打印启动信息
  RCLCPP_INFO(this->get_logger(), "=== DAOSN Robotics 障碍物检测节点已启动 ===");
  RCLCPP_INFO(this->get_logger(), "话题说明:");
  RCLCPP_INFO(this->get_logger(), "  /obstacle_detected (Float32MultiArray): 障碍物距离数组");
  RCLCPP_INFO(this->get_logger(), "    - 数组格式: [前方, 后方, 左侧, 右侧] (单位: 米)");
  RCLCPP_INFO(this->get_logger(), "    - 索引0: 前方障碍物距离 (0.0=无障碍物, >0.0=距离)");
  RCLCPP_INFO(this->get_logger(), "    - 索引1: 后方障碍物距离 (0.0=无障碍物, >0.0=距离)");
  RCLCPP_INFO(this->get_logger(), "    - 索引2: 左侧障碍物距离 (0.0=无障碍物, >0.0=距离)");
  RCLCPP_INFO(this->get_logger(), "    - 索引3: 右侧障碍物距离 (0.0=无障碍物, >0.0=距离)");
  RCLCPP_INFO(this->get_logger(), "  /obstacle_detected_bool (Bool): 是否检测到障碍物");
  RCLCPP_INFO(this->get_logger(), "  /status (String): 详细状态信息");

  logDetectionParameters();
}



void ObstacleDetectorNode::initializeMaskZones()
{
  mask_zones_.clear();

  try
  {
    // 屏蔽区域
    double front_mask_angle = mask_angle_ * M_PI / 180.0;         // 前方
    double front_mask_range = mask_range_ * M_PI / 180.0;         // ±10度范围
    mask_zones_.push_back({ front_mask_angle - front_mask_range,  // -10度
                            front_mask_angle + front_mask_range,  // +10度
                            0.05,                                 // 最小屏蔽距离
                            0.8,                                  // 最大屏蔽距离（可根据配件大小调整）
                            "正前方配件" });

    // // 屏蔽区域1：正前方配件（0° ± 10°，距离0.05-0.8米）
    // double front_mask_angle = 0.0;                                // 正前方0度
    // double front_mask_range = 10.0 * M_PI / 180.0;                // ±10度范围
    // mask_zones_.push_back({ front_mask_angle - front_mask_range,  // -10度
    //                         front_mask_angle + front_mask_range,  // +10度
    //                         0.05,                                 // 最小屏蔽距离
    //                         0.8,                                  // 最大屏蔽距离
    //                         "正前方配件" });

    // 如果需要屏蔽更多区域，可以添加额外的屏蔽区域
    // 例如：如果配件比较大，可以分别屏蔽左前方和右前方
    //
    // // 屏蔽区域2：左前方配件
    // mask_zones_.push_back({
    //   5.0 * M_PI / 180.0,   // 5度
    //   15.0 * M_PI / 180.0,  // 15度
    //   0.05,
    //   0.6,
    //   "左前方配件"
    // });

    // // 屏蔽区域3：右前方配件
    // mask_zones_.push_back({
    //   -15.0 * M_PI / 180.0,  // -15度
    //   -5.0 * M_PI / 180.0,   // -5度
    //   0.05,
    //   0.6,
    //   "右前方配件"
    // });
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(this->get_logger(), "读取屏蔽区域配置时出错: %s, 使用默认配置", e.what());
  }

  // 打印屏蔽区域信息
  if (!mask_zones_.empty())
  {
    RCLCPP_INFO(this->get_logger(), "已配置 %zu 个屏蔽区域:", mask_zones_.size());
    for (size_t i = 0; i < mask_zones_.size(); ++i)
    {
      const auto& zone = mask_zones_[i];
      RCLCPP_INFO(this->get_logger(), "  屏蔽区域%zu: %s, 角度范围: %.1f° - %.1f°, 距离范围: %.2f - %.2fm", i + 1,
                  zone.description.c_str(), zone.angle_start * 180.0 / M_PI, zone.angle_end * 180.0 / M_PI,
                  zone.min_distance, zone.max_distance);
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "未配置屏蔽区域");
  }
}

bool ObstacleDetectorNode::isInMaskZone(double angle, double distance)
{
  // 标准化角度
  angle = normalizeAngle(angle);

  for (const auto& zone : mask_zones_)
  {
    double start_angle = normalizeAngle(zone.angle_start);
    double end_angle = normalizeAngle(zone.angle_end);

    // 检查角度是否在屏蔽区域内
    bool angle_in_range = false;
    if (start_angle <= end_angle)
    {
      // 正常情况
      angle_in_range = (angle >= start_angle && angle <= end_angle);
    }
    else
    {
      // 跨越±π边界的情况
      angle_in_range = (angle >= start_angle || angle <= end_angle);
    }

    // 检查距离是否在屏蔽区域内
    bool distance_in_range = (distance >= zone.min_distance && distance <= zone.max_distance);

    if (angle_in_range && distance_in_range)
    {
      return true;  // 在屏蔽区域内
    }
  }

  return false;  // 不在任何屏蔽区域内
}

void ObstacleDetectorNode::logDetectionParameters()
{
  RCLCPP_INFO(this->get_logger(), "检测参数配置:");
  RCLCPP_INFO(this->get_logger(), "  机器人几何 - 长度: %.2fm, 宽度: %.2fm", robot_length_, robot_width_);
  RCLCPP_INFO(this->get_logger(), "  激光雷达位置 - x: %.2fm, y: %.2fm (相对于机器人左上角)", lidar_x_, lidar_y_);
  RCLCPP_INFO(this->get_logger(), "  检测距离 - 前: %.2fm, 后: %.2fm, 左: %.2fm, 右: %.2fm", front_detection_distance_,
              back_detection_distance_, left_detection_distance_, right_detection_distance_);
  RCLCPP_INFO(this->get_logger(), "  检测方式: 基于机器人矩形边界的精确检测");
}

void ObstacleDetectorNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{

  // 实时输出当前帧 LaserScan 的最小值和最大值（仅过滤无效数据）
  if (!msg->ranges.empty())
  {
    float min_valid = std::numeric_limits<float>::infinity();
    float max_valid = -std::numeric_limits<float>::infinity();
    bool has_valid = false;

    for (const auto& r : msg->ranges)
    {
      // 仅过滤无效数据（inf、-inf、NaN），保留所有数值
      if (!std::isfinite(r))
      {
        continue;
      }

      has_valid = true;
      // if (r < min_valid)
      // {
      //   min_valid = r;
      // }
      // if (r > max_valid)
      // {
      //   max_valid = r;
      // }
    }

    if (has_valid)
    {
      // RCLCPP_INFO(this->get_logger(),
      //             "LaserScan 有效距离: min=%.3f, max=%.3f (已过滤 inf/NaN)",
      //             min_valid, max_valid);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),
                  "LaserScan 当前帧没有有效距离数据 (所有点均为 inf/NaN)");
    }
  }

  if (!first_scan_received_)
  {
    first_scan_received_ = true;
    RCLCPP_INFO(this->get_logger(), "已接收到第一帧激光雷达数据");
    RCLCPP_INFO(this->get_logger(), "  扫描范围: %.2f - %.2f 弧度 (%.1f - %.1f 度)", msg->angle_min, msg->angle_max,
                msg->angle_min * 180.0 / M_PI, msg->angle_max * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  距离范围: %.2f - %.2f 米", msg->range_min, msg->range_max);
    RCLCPP_INFO(this->get_logger(), "  点数: %zu", msg->ranges.size());
  }

  // 检测障碍物
  current_detection_ = detectObstacles(msg);

  if (!first_scan_received_)
  {
    return;
  }

  // 发布检测结果
  publishObstacleStatus(current_detection_);
}

ObstacleDetection ObstacleDetectorNode::detectObstacles(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  ObstacleDetection detection;

  if (scan->ranges.empty())
  {
    return detection;
  }

  // 使用基于机器人矩形形状的检测方法
  // 检测前方障碍物
  detection.front = isObstacleInRobotDirection(scan, "front", front_detection_distance_);
  detection.front_distance = getMinObstacleDistanceInDirection(scan, "front", front_detection_distance_);
  
  // 检测后方障碍物
  detection.back = isObstacleInRobotDirection(scan, "back", back_detection_distance_);
  detection.back_distance = getMinObstacleDistanceInDirection(scan, "back", back_detection_distance_);
  
  // 检测左侧障碍物
  detection.left = isObstacleInRobotDirection(scan, "left", left_detection_distance_);
  detection.left_distance = getMinObstacleDistanceInDirection(scan, "left", left_detection_distance_);
  
  // 检测右侧障碍物
  detection.right = isObstacleInRobotDirection(scan, "right", right_detection_distance_);
  detection.right_distance = getMinObstacleDistanceInDirection(scan, "right", right_detection_distance_);

  // 更新最近障碍物信息
  std::vector<std::pair<std::string, double>> distances = {
    {"front", detection.front_distance},
    {"back", detection.back_distance},
    {"left", detection.left_distance},
    {"right", detection.right_distance}
  };

  for (const auto& dist : distances)
  {
    if (std::isfinite(dist.second) && dist.second < detection.closest_distance)
    {
      detection.closest_distance = dist.second;
      detection.closest_direction = dist.first;
    }
  }

  return detection;
}

double ObstacleDetectorNode::normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

std::string ObstacleDetectorNode::distanceArrayToString(const std::vector<float>& distances) const
{
  std::vector<std::string> detected_directions;
  const std::vector<std::string> direction_names = { "前方", "后方", "左侧", "右侧" };

  for (size_t i = 0; i < distances.size() && i < direction_names.size(); ++i)
  {
    if (distances[i] > 0.0f)
    {
      std::string distance_str = std::to_string(distances[i]);
      // 保留2位小数
      size_t decimal_pos = distance_str.find('.');
      if (decimal_pos != std::string::npos && decimal_pos + 3 < distance_str.length())
      {
        distance_str = distance_str.substr(0, decimal_pos + 3);
      }
      detected_directions.push_back(direction_names[i] + "(" + distance_str + "m)");
    }
  }

  if (detected_directions.empty())
    return "无障碍物";

  std::string result;
  for (size_t i = 0; i < detected_directions.size(); ++i)
  {
    if (i > 0)
      result += "+";
    result += detected_directions[i];
  }
  return result;
}

void ObstacleDetectorNode::publishObstacleStatus(const ObstacleDetection& detection)
{
  // 发布障碍物距离数组 (主要话题)
  auto obstacle_distances_msg = std_msgs::msg::Float32MultiArray();
  auto distance_array = detection.getDistanceArray();

  // 设置数组维度信息
  obstacle_distances_msg.layout.dim.resize(1);
  obstacle_distances_msg.layout.dim[0].label = "distances";
  obstacle_distances_msg.layout.dim[0].size = distance_array.size();
  obstacle_distances_msg.layout.dim[0].stride = distance_array.size();
  obstacle_distances_msg.layout.data_offset = 0;

  // 设置数据
  obstacle_distances_msg.data = distance_array;
  obstacle_detected_pub_->publish(obstacle_distances_msg);
  
  // 发布Bool状态
  auto obstacle_bool_msg = std_msgs::msg::Bool();
  obstacle_bool_msg.data = detection.hasObstacle();
  obstacle_bool_pub_->publish(obstacle_bool_msg);

  // 发布详细状态信息
  auto status_msg = std_msgs::msg::String();
  std::ostringstream oss;

  if (detection.hasObstacle())
  {
    oss << "检测到障碍物: ";
    std::vector<std::string> detected_directions;

    if (detection.front)
    {
      detected_directions.push_back("前方(" + std::to_string(detection.front_distance).substr(0, 4) + "m)");
    }
    if (detection.back)
    {
      detected_directions.push_back("后方(" + std::to_string(detection.back_distance).substr(0, 4) + "m)");
    }
    if (detection.left)
    {
      detected_directions.push_back("左侧(" + std::to_string(detection.left_distance).substr(0, 4) + "m)");
    }
    if (detection.right)
    {
      detected_directions.push_back("右侧(" + std::to_string(detection.right_distance).substr(0, 4) + "m)");
    }

    for (size_t i = 0; i < detected_directions.size(); ++i)
    {
      if (i > 0)
        oss << ", ";
      oss << detected_directions[i];
    }

    if (detection.closest_direction != "none")
    {
      oss << " | 最近: " << detection.closest_direction << "("
          << std::to_string(detection.closest_distance).substr(0, 4) << "m)";
    }

    // 添加距离数组信息
    oss << " | 距离数组: [";
    for (size_t i = 0; i < distance_array.size(); ++i)
    {
      if (i > 0)
        oss << ",";
      oss << std::fixed << std::setprecision(2) << distance_array[i];
    }
    oss << "] (" << distanceArrayToString(distance_array) << ")";
  }
  else
  {
    oss << "安全区域: 前:" << std::to_string(detection.front_distance).substr(0, 4) << "m "
        << "后:" << std::to_string(detection.back_distance).substr(0, 4) << "m "
        << "左:" << std::to_string(detection.left_distance).substr(0, 4) << "m "
        << "右:" << std::to_string(detection.right_distance).substr(0, 4) << "m "
        << "| 距离数组: [" << std::fixed << std::setprecision(2) << distance_array[0] << "," << distance_array[1] << ","
        << distance_array[2] << "," << distance_array[3] << "]";
  }

  status_msg.data = oss.str();
  obstacle_status_pub_->publish(status_msg);

  // 根据情况打印日志
  if (detection.hasObstacle())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", status_msg.data.c_str());
  }
  else
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", status_msg.data.c_str());
  }
}

std::pair<double, double> ObstacleDetectorNode::lidarPointToRobotCoords(double angle, double distance)
{
  // 将激光雷达坐标转换为机器人坐标系
  // 
  // ROS2激光雷达坐标系（右手坐标系）：
  // - 0° = +x方向（前方）
  // - 90° = +y方向（左侧，逆时针）
  // - -90° = -y方向（右侧）
  // - 180° = -x方向（后方）
  //
  // 机器人坐标系（左上角为原点）：
  // - x轴：向右为正
  // - y轴：向后为正
  // - 前方：y=0，后方：y=robot_length_
  //
  // 坐标转换映射：
  // - 激光雷达+x（0°，前方） → 机器人-y方向
  // - 激光雷达+y（90°，左侧） → 机器人-x方向
  // - 激光雷达-y（-90°，右侧） → 机器人+x方向
  // - 激光雷达-x（180°，后方） → 机器人+y方向
  
  double lidar_x_offset = distance * cos(angle);  // 激光雷达x分量
  double lidar_y_offset = distance * sin(angle);  // 激光雷达y分量
  
  // 转换到机器人坐标系
  double robot_point_x = lidar_x_ - lidar_y_offset;  // 激光雷达+y → 机器人-x
  double robot_point_y = lidar_y_ - lidar_x_offset;  // 激光雷达+x → 机器人-y
  
  return std::make_pair(robot_point_x, robot_point_y);
}

bool ObstacleDetectorNode::isPointInDetectionZone(double robot_x, double robot_y, const std::string& direction, double detection_distance)
{
  // 检查点是否在机器人的检测区域内
  // 机器人边界：左上角(0,0)，右下角(robot_width_, robot_length_)
  
  if (direction == "front")
  {
    // 前方检测区域：机器人前边界(y=0)向前延伸detection_distance
    return (robot_x >= 0 && robot_x <= robot_width_ && 
            robot_y >= -detection_distance && robot_y <= 0);
  }
  else if (direction == "back")
  {
    // 后方检测区域：机器人后边界(y=robot_length_)向后延伸detection_distance
    return (robot_x >= 0 && robot_x <= robot_width_ && 
            robot_y >= robot_length_ && robot_y <= robot_length_ + detection_distance);
  }
  else if (direction == "left")
  {
    // 左侧检测区域：机器人左边界向左延伸detection_distance
    return (robot_x >= -detection_distance && robot_x <= 0 && 
            robot_y >= 0 && robot_y <= robot_length_);
  }
  else if (direction == "right")
  {
    // 右侧检测区域：机器人右边界向右延伸detection_distance
    return (robot_x >= robot_width_ && robot_x <= robot_width_ + detection_distance && 
            robot_y >= 0 && robot_y <= robot_length_);
  }
  
  return false;
}

bool ObstacleDetectorNode::isObstacleInRobotDirection(const sensor_msgs::msg::LaserScan::SharedPtr scan, 
                                                      const std::string& direction, double detection_distance)
{
  if (scan->ranges.empty())
  {
    return false;
  }

  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {
    float range = scan->ranges[i];
    
    // 计算当前点的角度
    double angle = scan->angle_min + i * scan->angle_increment;
    
    // 检查是否在屏蔽区域内
    if (isInMaskZone(angle, static_cast<double>(range)))
    {
      continue;
    }
    
    // 将激光雷达点转换为机器人坐标系
    auto robot_coords = lidarPointToRobotCoords(angle, static_cast<double>(range));
    double robot_x = robot_coords.first;
    double robot_y = robot_coords.second;
    
    // 检查点是否在指定方向的检测区域内
    if (isPointInDetectionZone(robot_x, robot_y, direction, detection_distance))
    {
      return true;  // 发现障碍物
    }
  }
  
  return false;  // 未发现障碍物
}

double ObstacleDetectorNode::getMinObstacleDistanceInDirection(const sensor_msgs::msg::LaserScan::SharedPtr scan, 
                                                               const std::string& direction, double detection_distance)
{
  double min_distance = std::numeric_limits<double>::infinity();
  
  if (scan->ranges.empty())
  {
    return min_distance;
  }

  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {
    float range = scan->ranges[i];
  
    
    // 计算当前点的角度
    double angle = scan->angle_min + i * scan->angle_increment;
    
    // 检查是否在屏蔽区域内
    if (isInMaskZone(angle, static_cast<double>(range)))
    {
      continue;
    }
    
    // 将激光雷达点转换为机器人坐标系
    auto robot_coords = lidarPointToRobotCoords(angle, static_cast<double>(range));
    double robot_x = robot_coords.first;
    double robot_y = robot_coords.second;
    
    // 检查点是否在指定方向的检测区域内
    if (isPointInDetectionZone(robot_x, robot_y, direction, detection_distance))
    {
      // 计算从机器人边界到障碍物的距离
      double distance_from_robot_edge = 0.0;
      
      if (direction == "front")
      {
        distance_from_robot_edge = -robot_y;  // 前方：y=0到障碍物的距离
      }
      else if (direction == "back")
      {
        distance_from_robot_edge = robot_y - robot_length_;  // 后方：robot_length_到障碍物的距离
      }
      else if (direction == "left")
      {
        distance_from_robot_edge = -robot_x;
      }
      else if (direction == "right")
      {
        distance_from_robot_edge = robot_x - robot_width_;
      }
      
      if (distance_from_robot_edge >= 0)
      {
        min_distance = std::min(min_distance, distance_from_robot_edge);
      }
    }
  }
  
  return min_distance;
}

}  // namespace xline_obstacle_detector
