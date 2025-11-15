#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "xline_obstacle_detector/yaml_parser.hpp"
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace xline_obstacle_detector
{

// 障碍物方向索引定义
enum class ObstacleDirectionIndex : size_t
{
  FRONT = 0,  // 前方障碍物 (索引 0)
  BACK = 1,   // 后方障碍物 (索引 1)
  LEFT = 2,   // 左侧障碍物 (索引 2)
  RIGHT = 3,  // 右侧障碍物 (索引 3)
  COUNT = 4   // 总方向数
};

struct ObstacleDetection
{
  bool front = false;
  bool back = false;
  bool left = false;
  bool right = false;
  double front_distance = std::numeric_limits<double>::infinity();
  double back_distance = std::numeric_limits<double>::infinity();
  double left_distance = std::numeric_limits<double>::infinity();
  double right_distance = std::numeric_limits<double>::infinity();

  // 最近障碍物的方向和距离
  std::string closest_direction = "none";
  double closest_distance = std::numeric_limits<double>::infinity();

  // 获取障碍物距离数组 [前, 后, 左, 右]
  // 0.0 = 无障碍物，> 0.0 = 障碍物距离(米)
  std::vector<float> getDistanceArray() const
  {
    return {
      front ? static_cast<float>(front_distance) : 0.0f,  // 索引0: 前方
      back ? static_cast<float>(back_distance) : 0.0f,    // 索引1: 后方
      left ? static_cast<float>(left_distance) : 0.0f,    // 索引2: 左侧
      right ? static_cast<float>(right_distance) : 0.0f   // 索引3: 右侧
    };
  }

  // 是否检测到任何障碍物
  bool hasObstacle() const
  {
    return front || back || left || right;
  }
};

struct DetectionZone
{
  double distance_threshold;
  double angle_start;  // 起始角度（弧度）
  double angle_end;    // 结束角度（弧度）
  std::string name;
};

// 屏蔽区域结构体
struct MaskZone
{
  double angle_start;       // 起始角度（弧度）
  double angle_end;         // 结束角度（弧度）
  double min_distance;      // 最小屏蔽距离（可选，默认为0）
  double max_distance;      // 最大屏蔽距离（可选，默认为无穷大）
  std::string description;  // 屏蔽区域描述
};

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode();
  ~ObstacleDetectorNode() = default;

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  ObstacleDetection detectObstacles(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void publishObstacleStatus(const ObstacleDetection& detection);


  // 检查是否在屏蔽区域内
  bool isInMaskZone(double angle, double distance);

  // 基于机器人矩形形状的障碍物检测
  bool isObstacleInRobotDirection(const sensor_msgs::msg::LaserScan::SharedPtr scan, 
                                  const std::string& direction, double detection_distance);
  
  // 将激光雷达点转换为机器人坐标系
  std::pair<double, double> lidarPointToRobotCoords(double angle, double distance);
  
  // 检查点是否在机器人检测区域内
  bool isPointInDetectionZone(double robot_x, double robot_y, const std::string& direction, double detection_distance);
  
  // 获取指定方向的最小障碍物距离
  double getMinObstacleDistanceInDirection(const sensor_msgs::msg::LaserScan::SharedPtr scan, 
                                           const std::string& direction, double detection_distance);

  double normalizeAngle(double angle);

  // 初始化屏蔽区域
  void initializeMaskZones();

  void logDetectionParameters();

  // 将障碍物距离数组转换为字符串
  std::string distanceArrayToString(const std::vector<float>& distances) const;

  // ROS2 subscribers, publishers and timers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  // 使用Float32MultiArray发布障碍物距离数组
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_detected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_status_pub_;
  
  // 发布整体障碍物状态（是/否有障碍物）
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_bool_pub_;

  // Parameters
  double front_detection_distance_;
  double back_detection_distance_;
  double left_detection_distance_;
  double right_detection_distance_;


  std::string laser_topic_;

  double mask_angle_;
  double mask_range_;

  // 机器人几何参数
  double robot_length_;     // 机器人长度 (前后方向)
  double robot_width_;      // 机器人宽度 (左右方向)
  
  // 激光雷达位置 (相对于机器人左上角原点)
  double lidar_x_;          // 雷达x坐标 (向右为正)
  double lidar_y_;          // 雷达y坐标 (向前为正)


  // 屏蔽区域
  std::vector<MaskZone> mask_zones_;

  // Current state
  ObstacleDetection current_detection_;


  bool first_scan_received_;
};

}  // namespace xline_obstacle_detector