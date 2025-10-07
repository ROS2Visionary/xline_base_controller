#include "xline_localization/localization_node.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;    
using std::placeholders::_1;
using std::placeholders::_2;

namespace xline
{
namespace localization
{ 

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("localization", options)
{
  // 从 YAML 文件更新参数
  updateParameter();

  // 创建 IMU 订阅器（使用 SensorDataQoS 以兼容传感器发布器）
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu",
      rclcpp::SensorDataQoS(),
      std::bind(&LocalizationNode::imuCallback, this, _1));

  // 创建里程计订阅器
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::QoS{10},
      std::bind(&LocalizationNode::odomCallback, this, _1));

  // 创建反射板位置订阅器
  reflector_position_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/reflector_position",
      rclcpp::QoS{10},
      std::bind(&LocalizationNode::reflectorPositionCallback, this, _1));

  // 创建位姿发布器
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/robot_pose",
      rclcpp::QoS{10});

  // 创建姿态校准服务
  calibrate_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/calibrate_pose",
      std::bind(&LocalizationNode::calibratePoseCallback, this, _1, _2));

  // 初始化位姿消息
  reflector_pose_.header.frame_id = "map";
  robot_pose_.header.frame_id = "map";

  // 初始化数据源时间戳（使用节点时钟，设为0表示从未接收）
  last_odom_time_ = rclcpp::Time(0LL, RCL_ROS_TIME);
  last_reflector_time_ = rclcpp::Time(0LL, RCL_ROS_TIME);

  // 创建watchdog定时器（20Hz，与数据源同频）
  watchdog_timer_ = this->create_wall_timer(
      50ms,
      std::bind(&LocalizationNode::watchdogCallback, this));

  RCLCPP_INFO(get_logger(), "Localization 节点已启动");
  RCLCPP_INFO(get_logger(), "反射板到基座偏移: [%.3f, %.3f, %.3f]",
              reflector_to_base_offset_.x(),
              reflector_to_base_offset_.y(),
              reflector_to_base_offset_.z());
  RCLCPP_INFO(get_logger(), "订阅话题: /imu, /odom, /reflector_position");
  RCLCPP_INFO(get_logger(), "发布话题: /estimated_pose");
  RCLCPP_INFO(get_logger(), "服务: ~/calibrate_pose");
}

LocalizationNode::~LocalizationNode() = default;

void LocalizationNode::updateImu(const sensor_msgs::msg::Imu::ConstSharedPtr new_imu)
{
  std::scoped_lock<std::mutex> lock(imu_mutex_);
  imu_data_ = *new_imu;
  imu_updated_ = true;
}

bool LocalizationNode::getImu(sensor_msgs::msg::Imu & data)
{
  std::scoped_lock<std::mutex> lock(imu_mutex_);
  data = imu_data_;
  return imu_updated_;
}

void LocalizationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  updateImu(msg);
  RCLCPP_DEBUG(get_logger(), "收到 IMU 数据");
}

void LocalizationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 更新数据接收时间戳
  last_odom_time_ = this->now();

  // 提取机器人位置
  geometry_msgs::msg::Point robot_position = msg->pose.pose.position;

  // 如果正在收集数据,记录位置点
  {
    std::scoped_lock<std::mutex> lock(collection_mutex_);
    if (is_collecting_) {
      // 过滤无效数据(0,0)
      if (robot_position.x != 0.0 || robot_position.y != 0.0) {
        position_samples_.push_back({robot_position.x, robot_position.y});
        RCLCPP_DEBUG(get_logger(), "收集位置点: [%.3f, %.3f], 总数: %zu",
                     robot_position.x, robot_position.y, position_samples_.size());
      }
    }
  }

  if (!initialized_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "定位节点未初始化,请调用 ~/calibrate_pose 服务进行校准");
    // publishZeroPose();
    // return;
  }

  // 获取最新的IMU数据
  sensor_msgs::msg::Imu latest_imu;
  getImu(latest_imu);

  // 计算机器人姿态(基于IMU姿态差值)
  double imu_yaw = tf2::getYaw(latest_imu.orientation);
  double diff = imu_yaw - imu_initial_yaw_;
  diff = std::remainder(diff, 2 * M_PI);
  double robot_yaw = diff + robot_initial_yaw_;
  robot_yaw = std::remainder(robot_yaw, 2 * M_PI);

  // 根据里程计位置反推反射板位置
  // robot_position = reflector_position + reflector_to_base_offset (在机器人坐标系下)
  // 因此: reflector_position = robot_position - reflector_to_base_offset
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, robot_yaw);
  tf2::Transform global_to_base(
    q,
    tf2::Vector3(robot_position.x, robot_position.y, robot_position.z));

  // 计算反射板在全局坐标系的位置
  tf2::Transform global_to_reflector = global_to_base * reflector_to_base_tf_.inverse();

  // 更新反射板位姿
  {
    std::scoped_lock<std::mutex> lock(pose_mutex_);

    reflector_pose_.header.stamp = this->now();
    reflector_pose_.pose.position.x = global_to_reflector.getOrigin().getX();
    reflector_pose_.pose.position.y = global_to_reflector.getOrigin().getY();
    reflector_pose_.pose.position.z = global_to_reflector.getOrigin().getZ();
    reflector_pose_.pose.orientation = tf2::toMsg(q);
  }

  // 计算机器人位姿
  calcRobotPose();

  // 直接发布估计的位姿
  {
    std::scoped_lock<std::mutex> lock(pose_mutex_);
    pose_publisher_->publish(robot_pose_);
  }

  RCLCPP_DEBUG(get_logger(), "里程计位置: [%.3f, %.3f, %.3f], 姿态: %.3f rad",
               robot_position.x, robot_position.y, robot_position.z, robot_yaw);
}

void LocalizationNode::reflectorPositionCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // 更新数据接收时间戳
  last_reflector_time_ = this->now();

  {
    std::scoped_lock<std::mutex> lock(reflector_mutex_);
    reflector_position_ = *msg;
    reflector_updated_ = true;
  }

  // 如果正在收集数据,记录位置点
  {
    std::scoped_lock<std::mutex> lock(collection_mutex_);
    if (is_collecting_) {
      // 过滤无效数据(0,0)
      if (msg->point.x != 0.0 || msg->point.y != 0.0) {
        position_samples_.push_back({msg->point.x, msg->point.y});
        RCLCPP_DEBUG(get_logger(), "收集位置点: [%.3f, %.3f], 总数: %zu",
                     msg->point.x, msg->point.y, position_samples_.size());
      }
    }
  }

  if (!initialized_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "定位节点未初始化,请调用 ~/calibrate_pose 服务进行校准");
    // publishZeroPose();
    // return;
  }

  // 获取最新的IMU数据
  sensor_msgs::msg::Imu latest_imu;
  getImu(latest_imu);

  // 计算机器人姿态(基于IMU姿态差值)
  double imu_yaw = tf2::getYaw(latest_imu.orientation);
  double diff = imu_yaw - imu_initial_yaw_;
  diff = std::remainder(diff, 2 * M_PI);
  double robot_yaw = diff + robot_initial_yaw_;
  robot_yaw = std::remainder(robot_yaw, 2 * M_PI);

  // 更新反射板位姿
  {
    std::scoped_lock<std::mutex> lock(pose_mutex_);

    reflector_pose_.header.stamp = this->now();
    reflector_pose_.pose.position = msg->point;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, robot_yaw);
    reflector_pose_.pose.orientation = tf2::toMsg(q);
  }

  // 计算机器人位姿
  calcRobotPose();

  // 直接发布估计的位姿
  {
    std::scoped_lock<std::mutex> lock(pose_mutex_);
    pose_publisher_->publish(robot_pose_);
  }

  RCLCPP_DEBUG(get_logger(), "反射板位置: [%.3f, %.3f, %.3f]",
               msg->point.x, msg->point.y, msg->point.z);
}

void LocalizationNode::calcRobotPose()
{
  std::scoped_lock<std::mutex> lock(pose_mutex_);

  // 计算 global->reflector 变换
  tf2::Quaternion reflector_orientation;
  tf2::fromMsg(reflector_pose_.pose.orientation, reflector_orientation);
  tf2::Transform global_to_reflector(
    reflector_orientation,
    tf2::Vector3(reflector_pose_.pose.position.x,
                 reflector_pose_.pose.position.y,
                 reflector_pose_.pose.position.z));

  // 计算 global->base_link 变换
  tf2::Transform global_to_base = global_to_reflector * reflector_to_base_tf_;

  // 更新机器人位姿
  robot_pose_.header.stamp = this->now();
  robot_pose_.pose.orientation = reflector_pose_.pose.orientation;
  robot_pose_.pose.position.x = global_to_base.getOrigin().getX();
  robot_pose_.pose.position.y = global_to_base.getOrigin().getY();
  robot_pose_.pose.position.z = global_to_base.getOrigin().getZ();
}

void LocalizationNode::calibratePoseCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  RCLCPP_INFO(get_logger(), "开始姿态校准...");
  RCLCPP_INFO(get_logger(), "请让机器人沿直线移动一段距离(建议>0.5米)");

  // 开始收集位置数据
  {
    std::scoped_lock<std::mutex> lock(collection_mutex_);
    position_samples_.clear();
    is_collecting_ = true;
  }

  // 等待10秒收集数据
  RCLCPP_INFO(get_logger(), "正在收集位置数据,持续3秒...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // 停止收集
  {
    std::scoped_lock<std::mutex> lock(collection_mutex_);
    is_collecting_ = false;
  }

  RCLCPP_INFO(get_logger(), "收集完成,共收集 %zu 个位置点", position_samples_.size());

  // 拟合直线
  auto fitted_result = fitLine(position_samples_);

  if (!fitted_result.success) {
    response->success = false;
    response->message = "直线拟合失败";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  // 初始化位姿
  std::string error_msg;
  if (!initPose(fitted_result.start, fitted_result.end, error_msg)) {
    response->success = false;
    response->message = error_msg;
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return;
  }

  // 记录IMU初始方向
  sensor_msgs::msg::Imu latest_imu;
  if (getImu(latest_imu)) {
    imu_initial_yaw_ = tf2::getYaw(latest_imu.orientation);
  }

  initialized_ = true;

  response->success = true;
  response->message = "姿态校准成功";
  RCLCPP_INFO(get_logger(), "姿态校准完成 - 初始航向: %.3f rad (%.1f deg)",
              robot_initial_yaw_, robot_initial_yaw_ * 180.0 / M_PI);

  // 发布初始位姿
  {
    std::scoped_lock<std::mutex> lock(pose_mutex_);
    pose_publisher_->publish(robot_pose_);
  }
}

void LocalizationNode::updateParameter()
{
  // 使用ROS2包路径获取配置文件
  std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("xline_localization");
  std::string config_file = package_share_directory + "/config/localization_params.yaml";

  RCLCPP_INFO(this->get_logger(), "正在加载配置文件: %s", config_file.c_str());

  try {
    // 加载 YAML 文件
    YAML::Node config = YAML::LoadFile(config_file);

    // 检查是否有节点名称作为根节点
    YAML::Node params = config;
    if (config["localization"] && config["localization"]["ros__parameters"]) {
      params = config["localization"]["ros__parameters"];
    } else if (config["/**"] && config["/**"]["ros__parameters"]) {
      params = config["/**"]["ros__parameters"];
    }

    // 读取反射板到基座的偏移量(必须存在,不设默认值)
    if (!params["reflector_to_base_x"] || !params["reflector_to_base_y"] || !params["reflector_to_base_z"]) {
      throw std::runtime_error("配置文件中缺少必需参数: reflector_to_base_x/y/z");
    }

    reflector_to_base_offset_.x() = params["reflector_to_base_x"].as<double>();
    reflector_to_base_offset_.y() = params["reflector_to_base_y"].as<double>();
    reflector_to_base_offset_.z() = params["reflector_to_base_z"].as<double>();

    // 初始化反射板到基座的坐标转换
    reflector_to_base_tf_ = tf2::Transform(
      tf2::Quaternion().getIdentity(),
      tf2::Vector3(reflector_to_base_offset_.x(),
                   reflector_to_base_offset_.y(),
                   reflector_to_base_offset_.z()));

    RCLCPP_INFO(this->get_logger(), "参数加载成功:");
    RCLCPP_INFO(this->get_logger(), "  reflector_to_base_x: %.3f", reflector_to_base_offset_.x());
    RCLCPP_INFO(this->get_logger(), "  reflector_to_base_y: %.3f", reflector_to_base_offset_.y());
    RCLCPP_INFO(this->get_logger(), "  reflector_to_base_z: %.3f", reflector_to_base_offset_.z());

  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "解析 YAML 文件失败: %s", e.what());
    throw;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "加载参数失败: %s", e.what());
    throw;
  }
}

LocalizationNode::LineFitResult LocalizationNode::fitLine(
    const std::vector<std::vector<double>>& position_samples)
{
  LineFitResult result;
  result.success = false;

  // 检查输入数据是否有效
  if (position_samples.size() < 2) {
    RCLCPP_ERROR(get_logger(), "错误: 输入的数据点数量不足以拟合直线(至少需要2个点)");
    return result;
  }

  // 转换为 Point 类型
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto& vec : position_samples) {
    if (vec.size() != 2) {
      RCLCPP_ERROR(get_logger(), "错误: 每个数据点必须包含两个元素: x 和 y");
      return result;
    }
    geometry_msgs::msg::Point p;
    p.x = vec[0];
    p.y = vec[1];
    p.z = 0.0;
    points.push_back(p);
  }

  // 计算均值
  double sum_x = 0.0, sum_y = 0.0;
  for (const auto& p : points) {
    sum_x += p.x;
    sum_y += p.y;
  }
  double mean_x = sum_x / points.size();
  double mean_y = sum_y / points.size();

  // 计算斜率和截距(最小二乘法)
  double numerator = 0.0;
  double denominator = 0.0;
  for (const auto& p : points) {
    numerator += (p.x - mean_x) * (p.y - mean_y);
    denominator += (p.x - mean_x) * (p.x - mean_x);
  }

  double k = 0.0;
  double b = 0.0;
  bool vertical = false;
  double x_constant = 0.0;

  // 处理垂直线
  if (std::abs(denominator) < 1e-8) {
    vertical = true;
    x_constant = mean_x;
  } else {
    k = numerator / denominator;
    b = mean_y - k * mean_x;
  }

  // 获取第一个和最后一个点
  geometry_msgs::msg::Point first_point = points.front();
  geometry_msgs::msg::Point last_point = points.back();

  // 计算投影点
  result.start = projectPointToLine(first_point, k, b, vertical, x_constant);
  result.end = projectPointToLine(last_point, k, b, vertical, x_constant);
  result.success = true;

  RCLCPP_INFO(get_logger(), "直线拟合成功 - 起点: [%.3f, %.3f], 终点: [%.3f, %.3f]",
              result.start.x, result.start.y, result.end.x, result.end.y);

  return result;
}

geometry_msgs::msg::Point LocalizationNode::projectPointToLine(
    const geometry_msgs::msg::Point& point,
    double k, double b, bool vertical, double x_constant)
{
  geometry_msgs::msg::Point projected_point;
  projected_point.z = 0.0;

  if (vertical) {
    // 垂直线 x = x_constant
    projected_point.x = x_constant;
    projected_point.y = point.y;
  } else {
    // 直线方程: y = kx + b
    // 投影公式
    double denominator = k * k + 1;
    projected_point.x = (k * (point.y - b) + point.x) / denominator;
    projected_point.y = (k * k * point.y + k * point.x + b) / denominator;
  }

  return projected_point;
}

bool LocalizationNode::initPose(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& end,
    std::string& error_msg)
{
  // 计算移动方向向量
  Eigen::Vector3d p_start(start.x, start.y, start.z);
  Eigen::Vector3d p_end(end.x, end.y, end.z);
  Eigen::Vector3d diff = p_end - p_start;

  // 检查移动距离
  const double moved_dist = diff.head(2).norm();
  if (moved_dist < 0.05) {
    error_msg = "机器人移动距离过短: " + std::to_string(moved_dist) + "m (至少需要0.05m)";
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return false;
  }

  // 计算航向角
  const double yaw = std::atan2(diff.y(), diff.x());
  robot_initial_yaw_ = yaw;

  RCLCPP_INFO(get_logger(), "移动距离: %.3f m, 计算得到初始航向: %.3f rad (%.1f deg)",
              moved_dist, yaw, yaw * 180.0 / M_PI);

  // 初始化反射板位姿(使用终点位置)
  {
    std::scoped_lock<std::mutex> lock(pose_mutex_);

    reflector_pose_.header.stamp = this->now();
    reflector_pose_.pose.position = end;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    reflector_pose_.pose.orientation = tf2::toMsg(q);
  }

  // 计算机器人位姿
  calcRobotPose();

  return true;
}

void LocalizationNode::watchdogCallback()
{
  rclcpp::Time current_time = this->now();

  // 检查是否两个数据源都超时
  bool odom_timeout = false;
  bool reflector_timeout = false;
  double odom_elapsed = 0.0;
  double reflector_elapsed = 0.0;

  // 检查里程计超时（时间戳为0表示从未接收）
  if (last_odom_time_.nanoseconds() == 0) {
    odom_timeout = true;
    odom_elapsed = -1.0;  // 标记为未初始化
  } else {
    odom_elapsed = (current_time - last_odom_time_).seconds();
    odom_timeout = (odom_elapsed > data_timeout_threshold_);
  }

  // 检查反射器超时
  if (last_reflector_time_.nanoseconds() == 0) {
    reflector_timeout = true;
    reflector_elapsed = -1.0;  // 标记为未初始化
  } else {
    reflector_elapsed = (current_time - last_reflector_time_).seconds();
    reflector_timeout = (reflector_elapsed > data_timeout_threshold_);
  }

  if (odom_timeout && reflector_timeout) {
    // 两个数据源都超时，发布零位姿
    publishZeroPose();

    // 构建日志信息
    std::string odom_status = (odom_elapsed < 0) ? "未初始化" : std::to_string(odom_elapsed) + "s";
    std::string reflector_status = (reflector_elapsed < 0) ? "未初始化" : std::to_string(reflector_elapsed) + "s";

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "数据源超时 - 里程计: %s, 反射器: %s - 发布零位姿",
                         odom_status.c_str(), reflector_status.c_str());
  }
}

void LocalizationNode::publishZeroPose()
{
  geometry_msgs::msg::PoseStamped zero_pose;
  zero_pose.header.stamp = this->now();
  zero_pose.header.frame_id = "map";
  zero_pose.pose.position.x = 0.0;
  zero_pose.pose.position.y = 0.0;
  zero_pose.pose.position.z = 0.0;
  zero_pose.pose.orientation.w = 1.0;  // 单位四元数
  zero_pose.pose.orientation.x = 0.0;
  zero_pose.pose.orientation.y = 0.0;
  zero_pose.pose.orientation.z = 0.0;

  pose_publisher_->publish(zero_pose);
}

}  // namespace localization
}  // namespace xline

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(xline::localization::LocalizationNode)