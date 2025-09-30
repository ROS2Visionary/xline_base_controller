#pragma once

#include <memory>
#include <mutex>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

/**
 * Localization 定位节点
 *
 * 该节点负责融合多个传感器数据(IMU、激光反射板位置等)，
 * 计算机器人在全局坐标系中的实时位姿(位置、姿态)并发布。
 * 参考 daosnrs_localization 的实现方式。
 *
 * 订阅话题:
 * - /imu (sensor_msgs/Imu): IMU数据
 * - /reflector_position (geometry_msgs/PointStamped): 激光反射板位置
 *
 * 发布话题:
 * - /estimated_pose (geometry_msgs/PoseStamped): 机器人全局位姿
 *
 * 服务:
 * - ~/calibrate_pose (std_srvs/Trigger): 姿态校准服务
 */
namespace xline
{
namespace localization
{

class LocalizationNode : public rclcpp::Node
{
public:
  /**
   * 构造函数
   * @param options 节点选项
   */
  explicit LocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalizationNode() override;

private:
  // 订阅器
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr reflector_position_subscriber_;

  // 发布器
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

  // 服务
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;

  // 互斥锁
  std::mutex imu_mutex_;
  std::mutex reflector_mutex_;
  std::mutex pose_mutex_;

  // 反射板到机器人基座的坐标转换
  tf2::Transform reflector_to_base_tf_;
  Eigen::Vector3d reflector_to_base_offset_;  // 从参数读取

  // 姿态相关
  double robot_initial_yaw_{0.0};   // 机器人初始航向角
  double imu_initial_yaw_{0.0};     // IMU初始航向角
  bool initialized_{false};

  // 机器人位姿(base_link)
  geometry_msgs::msg::PoseStamped robot_pose_;

  // 反射板位姿
  geometry_msgs::msg::PoseStamped reflector_pose_;

  // 传感器数据缓存
  sensor_msgs::msg::Imu imu_data_;
  bool imu_updated_{false};

  geometry_msgs::msg::PointStamped reflector_position_;
  bool reflector_updated_{false};

  // 位置数据收集相关
  std::vector<std::vector<double>> position_samples_;  // 收集的位置点
  bool is_collecting_{false};  // 是否正在收集数据
  std::mutex collection_mutex_;

  // 直线拟合结果
  struct LineFitResult {
    bool success{false};
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point end;
  };

  /**
   * IMU 数据回调函数
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * 反射板位置回调函数
   */
  void reflectorPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  /**
   * 姿态校准服务回调
   * 用户调用此服务时,根据当前IMU和反射板数据校准机器人初始姿态
   */
  void calibratePoseCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * 获取IMU数据(线程安全)
   */
  bool getImu(sensor_msgs::msg::Imu & data);

  /**
   * 更新IMU数据(线程安全)
   */
  void updateImu(const sensor_msgs::msg::Imu::ConstSharedPtr new_imu);

  /**
   * 计算机器人位姿
   * 基于反射板位姿和反射板到基座的变换关系
   */
  void calcRobotPose();

  /**
   * 从 YAML 文件更新参数
   * 直接读取 YAML 配置文件并赋值给成员变量,不使用默认值
   */
  void updateParameter();

  /**
   * 拟合直线
   * 使用最小二乘法拟合收集的位置点
   */
  LineFitResult fitLine(const std::vector<std::vector<double>>& position_samples);

  /**
   * 计算点到直线的投影
   */
  geometry_msgs::msg::Point projectPointToLine(
    const geometry_msgs::msg::Point& point,
    double k, double b, bool vertical, double x_constant);

  /**
   * 初始化位姿
   * 根据拟合直线的起点和终点计算初始航向
   */
  bool initPose(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& end,
    std::string& error_msg);
};

}  // namespace localization
}  // namespace xline