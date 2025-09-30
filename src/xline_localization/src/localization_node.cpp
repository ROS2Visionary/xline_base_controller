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

  // 创建 IMU 订阅器
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu",
      rclcpp::QoS{10},
      std::bind(&LocalizationNode::imuCallback, this, _1));

  // 创建反射板位置订阅器
  reflector_position_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/reflector_position",
      rclcpp::QoS{10},
      std::bind(&LocalizationNode::reflectorPositionCallback, this, _1));

  // 创建位姿发布器
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/estimated_pose",
      rclcpp::QoS{10});

  // 创建姿态校准服务
  calibrate_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/calibrate_pose",
      std::bind(&LocalizationNode::calibratePoseCallback, this, _1, _2));

  // 初始化位姿消息
  reflector_pose_.header.frame_id = "map";
  robot_pose_.header.frame_id = "map";

  RCLCPP_INFO(get_logger(), "Localization 节点已启动");
  RCLCPP_INFO(get_logger(), "反射板到基座偏移: [%.3f, %.3f, %.3f]",
              reflector_to_base_offset_.x(),
              reflector_to_base_offset_.y(),
              reflector_to_base_offset_.z());
  RCLCPP_INFO(get_logger(), "订阅话题: /imu, /reflector_position");
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
  bool was_updated = imu_updated_;
  imu_updated_ = false;
  return was_updated;
}

void LocalizationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  updateImu(msg);
  RCLCPP_DEBUG(get_logger(), "收到 IMU 数据");
}

void LocalizationNode::reflectorPositionCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
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
      return;  // 收集数据期间不发布位姿
    }
  }

  if (!initialized_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "定位节点未初始化,请调用 ~/calibrate_pose 服务进行校准");
    return;
  }

  // 获取最新的IMU数据
  sensor_msgs::msg::Imu latest_imu;
  if (!getImu(latest_imu)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "无法获取 IMU 数据");
    return;
  }

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
  RCLCPP_INFO(get_logger(), "正在收集位置数据,持续10秒...");
  rclcpp::sleep_for(std::chrono::seconds(10));

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
  // 使用相对路径
  std::string config_file = "src/xline_localization/config/localization_params.yaml";

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

}  // namespace localization
}  // namespace xline

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(xline::localization::LocalizationNode)