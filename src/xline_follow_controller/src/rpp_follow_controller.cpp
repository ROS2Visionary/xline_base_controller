#include "xline_follow_controller/rpp_follow_controller.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>

namespace xline
{
namespace follow_controller
{

RPPController::RPPController()
  : initialized_(false)
  , goal_reached_(false)
  , max_error_(0.0)
  , avg_error_(0.0)
  , error_sum_(0.0)
  , error_count_(0)
  , path_length_(0.0)
  , traversed_distance_(0.0)
  , remaining_distance_(0.0)
  , enable_grid_map_(false)  // 默认不启用栅格图功能
  , grid_resolution_(0.01)   // 1厘米/像素
  , grid_width_(10.0)        // 默认10x10米的地图
  , grid_height_(10.0)
  , grid_map_path_("/home/daosn_robotics/zyq_ws/rpp_grid_map")  // 默认保存路径
  , previous_angular_vel_(0.0)
  , predicted_angular_vel_(0.0)
  , lowpass_angular_vel_filter_gain_(0.7)  // 滤波增益默认值
  , angular_vel_history_size_(5)           // 保留最近5个周期的角速度
  , angle_to_path_prev_(0.0)
  , lookahead_dist_prev_(0.0)
  , second_order_filter_(2.0, 0.7)
  , need_yaw_prealign_(false)             // 是否需要航向预对准
  , yaw_prealign_done_(false)             // 航向预对准是否完成
  , target_yaw_(0.0)                      // 圆形路径的目标航向
  , circle_entry_x_(0.0)                  // 圆形路径切入点x坐标
  , circle_entry_y_(0.0)                  // 圆形路径切入点y坐标
  , collect_positions_for_circle_(false)  // 是否收集位置用于半径计算
  , target_radius_(0.0)                   // 目标半径
  , target_center_x_(0.0)                 // 目标圆心x坐标
  , target_center_y_(0.0)                 // 目标圆心y坐标
  , position_collect_interval_(0.1)       // 位置收集间隔(秒)
  , robot_center_fitted_x_(0.0)           // 机器人中心拟合圆心x坐标
  , robot_center_fitted_y_(0.0)           // 机器人中心拟合圆心y坐标
  , robot_center_fitted_radius_(0.0)      // 机器人中心拟合半径
  , robot_center_fit_success_(false)      // 机器人中心拟合是否成功
  , back_follow_(false)                   // 后退模式默认关闭
{
  RCLCPP_INFO(get_logger(), "RPPController实例已创建，正在初始化...");
  initialize();
  updateParameters("/config/rpp_curve.yaml");
}

RPPController::~RPPController()
{
  RCLCPP_INFO(get_logger(), "RPPController实例已销毁");
}

void RPPController::initialize()
{
  if (!initialized_)
  {
    // 计算时间步长 - 更高频率以提高精度
    d_t_ = 1.0 / 18.0;  // 更新频率为18Hz

    // 创建栅格图保存目录
    if (enable_grid_map_)
    {
      std::filesystem::create_directories(grid_map_path_);
      RCLCPP_INFO(get_logger(), "栅格图将保存到: %s", grid_map_path_.c_str());
    }

    initialized_ = true;
    RCLCPP_INFO(get_logger(), "控制器初始化完成，控制周期: %s", (std::to_string(d_t_) + "s").c_str());
  }
}

void RPPController::updateParameters(std::string file_path)
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("xline_follow_controller");
  std::string config_file_path = package_share_directory + file_path;
  xline::YamlParser::YamlParser parser(config_file_path);

  // 基本参数 - 曲率和接近约束
  regulated_min_radius_ = parser.getParameter<double>("regulated_min_radius");
  approach_dist_ = parser.getParameter<double>("approach_dist");
  approach_min_v_ = parser.getParameter<double>("approach_min_v");

  goal_dist_tol_ = parser.getParameter<double>("goal_dist_tol");
  rotate_tol_ = parser.getParameter<double>("rotate_tol");

  lookahead_time_ = parser.getParameter<double>("lookahead_time");
  min_lookahead_dist_ = parser.getParameter<double>("min_lookahead_dist");
  max_lookahead_dist_ = parser.getParameter<double>("max_lookahead_dist");

  max_v_ = parser.getParameter<double>("max_v");
  min_v_ = parser.getParameter<double>("min_v");
  max_v_inc_ = parser.getParameter<double>("max_v_inc");
  linear_speed_ = parser.getParameter<double>("linear_speed");

  max_w_ = parser.getParameter<double>("max_w");
  min_w_ = parser.getParameter<double>("min_w");
  max_w_inc_ = parser.getParameter<double>("max_w_inc");

  radius_offset_ = parser.getParameter<double>("radius_offset");
  lowpass_angular_vel_filter_gain_ = parser.getParameter<double>("lowpass_angular_vel_filter_gain");
  smoothing_type_ = parser.getParameter<std::string>("smoothing_type");

  angular_vel_history_size_ = parser.getParameter<int>("angular_vel_history_size");

  pos_cutoff_freq = parser.getParameter<double>("pos_cutoff_freq");
  pos_sample_rate = parser.getParameter<double>("pos_sample_rate");
  pos_output_limit = parser.getParameter<double>("pos_output_limit");
  pos_rate_limit = parser.getParameter<double>("pos_rate_limit");

  pos_use_biquad_cascade_ = parser.getParameter<bool>("pos_use_biquad_cascade");
  pos_use_biquad_cascade_filter_ = parser.getParameter<bool>("pos_use_biquad_cascade_filter");

  angle_cutoff_freq = parser.getParameter<double>("angle_cutoff_freq");
  angle_sample_rate = parser.getParameter<double>("angle_sample_rate");
  angle_output_limit_rate = parser.getParameter<double>("angle_output_limit_rate");
  angle_rate_limit = parser.getParameter<double>("angle_rate_limit");

  angle_use_biquad_cascade_ = parser.getParameter<bool>("angle_use_biquad_cascade");
  angle_use_biquad_cascade_filter_ = parser.getParameter<bool>("angle_use_biquad_cascade_filter");
  angle_use_offset_limit_ = parser.getParameter<bool>("angle_use_offset_limit");
  angle_output_offset_ = parser.getParameter<double>("angle_output_offset");

  start_deviation_factor_ = parser.getParameter<double>("start_deviation_factor");
  end_deviation_factor_ = parser.getParameter<double>("end_deviation_factor");
  deviation_rate_ = parser.getParameter<double>("deviation_rate");

  enable_grid_map_ = parser.getParameter<bool>("enable_grid_map");

  radius_log_file_path_ = parser.getParameter<std::string>("radius_log_file_path");

  x_offset_ = parser.getParameter<double>("x_offset");
  y_offset_ = parser.getParameter<double>("y_offset");

  collect_positions_for_circle_ = parser.getParameter<bool>("collect_positions_for_circle");

  low_speed_mode_ = parser.getParameter<bool>("low_speed_mode");

  addOffsetPoint(x_offset_, y_offset_, "record");

  // 读取二阶平滑器参数
  angular_smoother_freq_ = parser.getParameter<double>("smoother_freq");
  angular_smoother_damping_ = parser.getParameter<double>("smoother_damping");

  // 更新二阶平滑器参数
  second_order_filter_.setParameters(angular_smoother_freq_, angular_smoother_damping_);

  // 记录参数更新信息
  std::stringstream ss;
  ss << "参数已更新: " << std::endl
     << "  最小曲率半径: " << regulated_min_radius_ << "m" << std::endl
     << "  接近距离: " << approach_dist_ << "m" << std::endl
     << "  接近最小速度: " << approach_min_v_ << "m/s" << std::endl
     << "  目标距离容忍度: " << goal_dist_tol_ << "m" << std::endl
     << "  旋转容忍度: " << rotate_tol_ << "rad" << std::endl
     << "  前瞻时间: " << lookahead_time_ << "s" << std::endl
     << "  最小前瞻距离: " << min_lookahead_dist_ << "m" << std::endl
     << "  最大前瞻距离: " << max_lookahead_dist_ << "m" << std::endl
     << "  最大线速度: " << max_v_ << "m/s" << std::endl
     << "  基准线速度: " << linear_speed_ << "m/s" << std::endl
     << "  最大角速度: " << max_w_ << "rad/s";

  RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
}


void RPPController::setAngleRange(double start_angle, double end_anngle)
{
  circle_start_angle = start_angle;
  circle_end_angle = end_anngle;
  circle_total_angle = std::abs(circle_end_angle - circle_start_angle) + 0.6 * M_PI;
}

bool RPPController::setPlanForCircle(double circle_center_x, double circle_center_y, double circle_radius,
                                     const geometry_msgs::msg::PoseStamped& robot_pose)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化，请先调用initialize()");
    return false;
  }

  // 检查输入参数有效性
  if (circle_radius <= 0.0)
  {
    RCLCPP_ERROR(get_logger(), "圆半径必须为正值");
    return false;
  }

  updateParameters("/config/rpp_circle.yaml");

  // 圆形路径不使用后退模式
  back_follow_ = false;

  if (!low_speed_mode_)
  {
    if (circle_radius < 0.5)
    {
      // 小半径：高精度，低速度
      min_lookahead_dist_ = 0.15;
      max_lookahead_dist_ = 0.15;
      max_v_ = 0.08;
      min_v_ = 0.08;
      linear_speed_ = 0.08;
    }
    else if (circle_radius < 0.8)
    {
      min_lookahead_dist_ = 0.17;
      max_lookahead_dist_ = 0.17;
      max_v_ = 0.10;
      min_v_ = 0.10;
      linear_speed_ = 0.10;
    }
    else if (circle_radius < 1.2)
    {
      // 较大半径：效率优先
      min_lookahead_dist_ = 0.21;
      max_lookahead_dist_ = 0.21;
      max_v_ = 0.11;
      min_v_ = 0.11;
      linear_speed_ = 0.11;
    }
    else
    {
      // 大半径：高速运行
      min_lookahead_dist_ = 0.17;
      max_lookahead_dist_ = 0.17;
      max_v_ = 0.11;
      min_v_ = 0.11;
      linear_speed_ = 0.11;
    }
  }

  circle_radius = circle_radius + radius_offset_;
  is_circle_path = true;

  // 保存目标圆信息
  target_center_x_ = circle_center_x;
  target_center_y_ = circle_center_y;
  target_radius_ = circle_radius;

  // 重置拟合结果
  robot_center_fitted_x_ = 0.0;
  robot_center_fitted_y_ = 0.0;
  robot_center_fitted_radius_ = 0.0;
  robot_center_fit_success_ = false;

  // 重置喷墨口拟合结果
  for (auto& offset_point : offset_points_)
  {
    offset_point.fitted_center_x = 0.0;
    offset_point.fitted_center_y = 0.0;
    offset_point.fitted_radius = 0.0;
    offset_point.fit_success = false;
    offset_point.positions.clear();
  }

  // 用于圆形路径的角度累计
  last_yaw_initialized_ = false;
  last_yaw_ = 0.0;
  accumulated_angle_ = 0.0;
  angle_debug_counter_ = 0;

  circle_center_x_ = circle_center_x;
  circle_center_y_ = circle_center_y;
  circle_radius_ = circle_radius;
  baseline_angular_velocity_for_circle_ = min_v_ / circle_radius;

  // 初始化位置收集
  collected_positions_.clear();
  last_position_collect_time_ = this->now();

  // 初始化喷墨口偏置点收集
  if (!offset_points_.empty())
  {
    for (auto& offset_point : offset_points_)
    {
      offset_point.positions.clear();
    }
    RCLCPP_INFO(get_logger(), "开始收集机器人中心和%zu个喷墨口的位置数据", offset_points_.size());
    RCLCPP_INFO(get_logger(), "目标圆心: (%.3f, %.3f), 目标半径: %.3f m", target_center_x_, target_center_y_,
                target_radius_);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "开始收集机器人中心位置用于半径计算");
    RCLCPP_INFO(get_logger(), "目标圆心: (%.3f, %.3f), 目标半径: %.3f m", target_center_x_, target_center_y_,
                target_radius_);
  }

  try
  {
    // 计算机器人到圆心的距离
    double dx = circle_center_x - robot_pose.pose.position.x;
    double dy = circle_center_y - robot_pose.pose.position.y;
    double distance_to_center = std::hypot(dx, dy);

    // 如果机器人在圆内，使用不同的策略
    bool robot_inside_circle = (distance_to_center < circle_radius);

    // 创建新路径
    nav_msgs::msg::Path circle_path;
    circle_path.header.frame_id = "world";
    circle_path.header.stamp = this->now();

    // 计算切入点
    // 机器人位置和圆心连线上的点，与圆的交点为切入目标
    double entry_x, entry_y;

    if (robot_inside_circle)
    {
      // 机器人在圆内，直接向外移动到圆上
      double angle_to_center = atan2(dy, dx);
      entry_x = circle_center_x - circle_radius * cos(angle_to_center);
      entry_y = circle_center_y - circle_radius * sin(angle_to_center);
    }
    else
    {
      // 计算切线点（左切，确保左转）
      double angle_to_center = atan2(dy, dx);
      double distance_ratio = circle_radius / distance_to_center;

      // 确保不超过1.0，避免数值问题
      distance_ratio = std::min(distance_ratio, 0.99);

      // 计算切线角度（左切）
      double tangent_angle = angle_to_center + asin(distance_ratio);

      // 计算从机器人到切点的距离
      double distance_to_tangent = distance_to_center * cos(asin(distance_ratio));

      // 计算切点坐标
      entry_x = robot_pose.pose.position.x + distance_to_tangent * cos(tangent_angle);
      entry_y = robot_pose.pose.position.y + distance_to_tangent * sin(tangent_angle);
    }

    // 创建从机器人到切入点的路径段
    // 1. 添加机器人当前位置
    circle_path.poses.push_back(robot_pose);

    // 3. 添加切入点
    geometry_msgs::msg::PoseStamped entry_pose;
    entry_pose.header = robot_pose.header;
    entry_pose.pose.position.x = entry_x;
    entry_pose.pose.position.y = entry_y;
    entry_pose.pose.position.z = robot_pose.pose.position.z;

    // 计算切入点处的切线方向（沿圆的切线，确保左转）
    double tangent_x = -(entry_y - circle_center_y);  // 逆时针90度旋转，确保左转
    double tangent_y = (entry_x - circle_center_x);
    double tangent_length = std::hypot(tangent_x, tangent_y);
    if (tangent_length > 1e-6)
    {
      tangent_x /= tangent_length;
      tangent_y /= tangent_length;
    }

    double entry_yaw = atan2(tangent_y, tangent_x);
    target_yaw_ = entry_yaw;
    tf2::Quaternion entry_q;
    entry_q.setRPY(0, 0, entry_yaw);
    entry_pose.pose.orientation = tf2::toMsg(entry_q);

    circle_path.poses.push_back(entry_pose);

    // 4. 生成圆周路径（逆时针，左转）
    int num_circle_points = 1500;  // 调整点数以适应圆形路径
    double start_angle = atan2(entry_y - circle_center_y, entry_x - circle_center_x);
    double total_angle = circle_total_angle;

    for (int i = 1; i <= num_circle_points; i++)
    {
      double angle = start_angle + i * (total_angle / num_circle_points);

      geometry_msgs::msg::PoseStamped circle_pose;
      circle_pose.header = robot_pose.header;
      circle_pose.pose.position.x = circle_center_x + circle_radius * cos(angle);
      circle_pose.pose.position.y = circle_center_y + circle_radius * sin(angle);
      circle_pose.pose.position.z = robot_pose.pose.position.z;

      // 计算切线方向（沿圆的切线）
      double tangent_direction = angle + M_PI / 2;  // 逆时针方向的切线
      tf2::Quaternion circle_q;
      circle_q.setRPY(0, 0, tangent_direction);
      circle_pose.pose.orientation = tf2::toMsg(circle_q);

      circle_path.poses.push_back(circle_pose);
    }

    // 调用原始的setPlan方法处理这个路径
    RCLCPP_INFO(get_logger(), "圆形路径已生成 - 圆心: (%.3f, %.3f), 半径: %.3f m, 点数: %zu", circle_center_x,
                circle_center_y, circle_radius, circle_path.poses.size());

    return setPlan(circle_path);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "设置圆形路径时发生异常: %s", e.what());
    return false;
  }
  catch (...)
  {
    RCLCPP_ERROR(get_logger(), "设置圆形路径时发生未知异常");
    return false;
  }
}

bool RPPController::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化，请先调用initialize()");
    return false;
  }

  if (orig_global_plan.poses.empty())
  {
    RCLCPP_ERROR(get_logger(), "收到空路径，无法设置计划");
    return false;
  }

  RCLCPP_INFO(get_logger(), "设置新计划，包含 %zu 个点", orig_global_plan.poses.size());

  // 重置后退模式
  back_follow_ = false;

  waiting_ = true;
  // 重置误差统计
  max_error_ = 0.0;
  avg_error_ = 0.0;
  error_sum_ = 0.0;
  error_count_ = 0;
  current_lateral_error_ = 0.0;  // 初始化当前横向误差

  // 重置二阶平滑器
  second_order_filter_.reset();

  previous_angular_vel_ = 0.0;
  angular_vel_history_.clear();

  // 确保设置新路径时重置目标达成状态
  goal_reached_ = false;

  sg_x_filter_.reset(5, 2);
  sg_y_filter_.reset(5, 2);
  h_x_filter.reset(5, 3.0);
  h_y_filter.reset(5, 3.0);

  pos_x_filter_.reset();
  pos_x_filter_.initialize(pos_cutoff_freq, pos_sample_rate, pos_output_limit);
  pos_x_filter_.setLimits(pos_output_limit, pos_rate_limit, true);
  pos_x_filter_.use_biquad_cascade_ = pos_use_biquad_cascade_;

  pos_y_filter_.reset();
  pos_y_filter_.initialize(pos_cutoff_freq, pos_sample_rate, pos_output_limit);
  pos_y_filter_.setLimits(pos_output_limit, pos_rate_limit, true);
  pos_y_filter_.use_biquad_cascade_ = pos_use_biquad_cascade_;

  angle_vel_filter_.reset();
  angle_vel_filter_.initialize(angle_cutoff_freq, angle_sample_rate,
                               angle_output_limit_rate * baseline_angular_velocity_for_circle_);
  angle_vel_filter_.setLimits(angle_output_limit_rate * baseline_angular_velocity_for_circle_, angle_rate_limit, true);
  angle_vel_filter_.setOffsetLimit(angle_output_offset_, angle_use_offset_limit_);
  angle_vel_filter_.use_biquad_cascade_ = angle_use_biquad_cascade_;

  // 重置索引
  last_closest_idx = 0;

  // 添加路径平滑和细分
  if (orig_global_plan.poses.size() > 2)
  {
    RCLCPP_INFO(get_logger(), "高精度模式下执行路径平滑和细分...");

    nav_msgs::msg::Path smoothed_path;
    smoothed_path.header = orig_global_plan.header;

    // 保留起点
    smoothed_path.poses.push_back(orig_global_plan.poses.front());

    // 平滑和细分中间点
    for (size_t i = 0; i < orig_global_plan.poses.size() - 1; ++i)
    {
      const auto& current = orig_global_plan.poses[i];
      const auto& next = orig_global_plan.poses[i + 1];

      // 计算当前线段长度
      double segment_length =
          std::hypot(next.pose.position.x - current.pose.position.x, next.pose.position.y - current.pose.position.y);

      // 如果线段过长，进行细分
      if (segment_length > 0.003)
      {
        int num_subdivisions = std::ceil(segment_length / 0.002);  // 每2mm一个点

        for (int j = 1; j < num_subdivisions; ++j)
        {
          double ratio = static_cast<double>(j) / num_subdivisions;

          geometry_msgs::msg::PoseStamped intermediate_pose;
          intermediate_pose.header = current.header;

          // 线性插值位置
          intermediate_pose.pose.position.x =
              current.pose.position.x + ratio * (next.pose.position.x - current.pose.position.x);
          intermediate_pose.pose.position.y =
              current.pose.position.y + ratio * (next.pose.position.y - current.pose.position.y);
          intermediate_pose.pose.position.z =
              current.pose.position.z + ratio * (next.pose.position.z - current.pose.position.z);

          // 计算朝向
          double yaw =
              atan2(next.pose.position.y - current.pose.position.y, next.pose.position.x - current.pose.position.x);
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          intermediate_pose.pose.orientation = tf2::toMsg(q);

          smoothed_path.poses.push_back(intermediate_pose);
        }
      }

      // 添加原始下一个点（除了最后一个循环，会在循环外添加终点）
      if (i < orig_global_plan.poses.size() - 2)
      {
        smoothed_path.poses.push_back(next);
      }
    }

    // 保留终点
    smoothed_path.poses.push_back(orig_global_plan.poses.back());

    RCLCPP_INFO(get_logger(), "路径优化完成: 原始点数=%zu, 优化后点数=%zu", orig_global_plan.poses.size(),
                smoothed_path.poses.size());

    // 使用优化后的路径
    global_plan_ = smoothed_path;
  }
  else
  {
    // 直接使用原始路径
    global_plan_ = orig_global_plan;
  }

  // 验证路径的有效性
  bool has_invalid_points = false;
  for (size_t i = 0; i < global_plan_.poses.size(); ++i)
  {
    if (!std::isfinite(global_plan_.poses[i].pose.position.x) || !std::isfinite(global_plan_.poses[i].pose.position.y))
    {
      RCLCPP_ERROR(get_logger(), "路径包含无效点 #%zu: (%f, %f)", i, global_plan_.poses[i].pose.position.x,
                   global_plan_.poses[i].pose.position.y);
      has_invalid_points = true;
    }
  }

  if (has_invalid_points)
  {
    RCLCPP_WARN(get_logger(), "路径包含无效点，将尝试清理");

    // 清理无效点
    std::vector<geometry_msgs::msg::PoseStamped> valid_poses;
    valid_poses.reserve(global_plan_.poses.size());

    for (const auto& pose : global_plan_.poses)
    {
      if (std::isfinite(pose.pose.position.x) && std::isfinite(pose.pose.position.y) &&
          std::isfinite(pose.pose.position.z))
      {
        valid_poses.push_back(pose);
      }
    }

    if (valid_poses.empty())
    {
      RCLCPP_ERROR(get_logger(), "清理后路径为空，无法继续");
      return false;
    }

    global_plan_.poses = valid_poses;
    RCLCPP_INFO(get_logger(), "清理后路径点数: %zu", global_plan_.poses.size());
  }

  // 计算路径总长度
  path_length_ = 0.0;
  for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i)
  {
    path_length_ += std::hypot(global_plan_.poses[i + 1].pose.position.x - global_plan_.poses[i].pose.position.x,
                               global_plan_.poses[i + 1].pose.position.y - global_plan_.poses[i].pose.position.y);
  }

  remaining_distance_ = path_length_;
  traversed_distance_ = 0.0;

  RCLCPP_INFO(get_logger(), "计算路径长度: %s", (std::to_string(path_length_) + "m").c_str());

  // 更新目标信息
  const auto& goal_pose = global_plan_.poses.back();
  goal_x_ = goal_pose.pose.position.x;
  goal_y_ = goal_pose.pose.position.y;
  goal_theta_ = tf2::getYaw(goal_pose.pose.orientation);
  goal_reached_ = false;

  // 记录目标信息
  RCLCPP_INFO(
      get_logger(), "目标位置: (%s",
      (std::to_string(goal_x_) + ", " + std::to_string(goal_y_) + ", " + std::to_string(goal_theta_) + ")").c_str());

  // 初始化当前曲率
  current_curvature_ = 0.0;

  // 在成功设置全局路径后，初始化栅格图并绘制路径
  if (enable_grid_map_)
  {
    initializeGridMap(global_plan_);
  }

  return true;
}

bool RPPController::isGoalReached()
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化");
    return false;
  }

  if (goal_reached_)
  {
    static bool reported = false;
    if (!reported)
    {
      RCLCPP_INFO(get_logger(), "目标已到达 - 最大误差: %s",
                  (std::to_string(max_error_) + "m, 平均误差: " + std::to_string(avg_error_) + "m").c_str());
      reported = true;
    }
    return true;
  }

  return false;
}

void RPPController::setBackFollow(bool back)
{
  back_follow_ = back;
  need_yaw_prealign_ = true;
  yaw_prealign_done_ = false;
  RCLCPP_INFO(get_logger(), "设置后退模式: %s", back ? "启用" : "禁用");
}

double RPPController::getLookAheadDistance(double speed)
{
  // 基础前瞻距离计算
  double base_lookahead_dist = std::fabs(speed) * lookahead_time_;

  // 基于当前横向误差动态调整前瞻距离
  // 注意：需要在类中添加一个current_lateral_error_成员变量来跟踪当前横向误差
  double error_factor = 1.0;
  if (std::fabs(current_lateral_error_) > 0.1)
  {
    // 误差大时减小前瞻距离，使控制器更专注于减小当前误差
    error_factor = 1.0 - std::min(0.3, std::fabs(current_lateral_error_) * 0.5);
  }

  // 应用误差因子
  double lookahead_dist = base_lookahead_dist * error_factor;

  // 限制在最小/最大值范围内
  lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);

  // 根据曲率进一步调整
  if (std::fabs(current_curvature_) > 5.0)
  {
    // 高曲率情况下减小前瞻距离以提高精度
    lookahead_dist = std::max(min_lookahead_dist_, lookahead_dist * 0.8);
  }

  // 近目标时调整
  if (remaining_distance_ < lookahead_dist * 2.0)
  {
    lookahead_dist = std::max(min_lookahead_dist_, remaining_distance_ / 2.0);
  }

  return lookahead_dist;
}

bool RPPController::shouldRotateToPath(double angle_to_path, double tolerance)
{
  // 检查是否需要旋转以对齐路径
  double angle_threshold = tolerance > 0.0 ? tolerance : rotate_tol_;
  return std::fabs(angle_to_path) > angle_threshold;
}

double RPPController::regularizeAngle(double angle)
{
  // 将角度规范化到[-π, π]区间
  return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

bool RPPController::shouldRotateToGoal(const geometry_msgs::msg::PoseStamped& current_pose,
                                       const geometry_msgs::msg::PoseStamped& goal_pose)
{
  // 检查是否足够接近目标以开始旋转到最终方向
  double distance_to_goal = std::hypot(current_pose.pose.position.x - goal_pose.pose.position.x,
                                       current_pose.pose.position.y - goal_pose.pose.position.y);

  return distance_to_goal < goal_dist_tol_;
}

double RPPController::linearRegularization(double current_velocity, double desired_velocity)
{
  // 计算速度增量
  double velocity_increment = desired_velocity - current_velocity;

  // 限制加速度 - 使用小的加速度限制以实现平滑运动
  if (std::fabs(velocity_increment) > max_v_inc_)
    velocity_increment = std::copysign(max_v_inc_, velocity_increment);

  // 应用增量
  double command_velocity = current_velocity + velocity_increment;

  // 限制最大/最小速度
  if (std::fabs(command_velocity) > max_v_)
    command_velocity = std::copysign(max_v_, command_velocity);
  else if (std::fabs(command_velocity) < min_v_ && desired_velocity != 0.0)
    command_velocity = std::copysign(min_v_, command_velocity);

  return command_velocity;
}

double RPPController::angularRegularization(double current_angular_vel, double desired_angular_vel)
{
  // 限制角速度不超过最大值
  double max_allowed_w = max_w_;

  // 根据曲率调整最大允许角速度
  if (std::fabs(desired_angular_vel) > max_w_ * 0.8)
  {
    // 高曲率情况下降低最大角速度
    max_allowed_w = max_w_ * 0.8;
  }

  // 限制期望角速度
  if (std::fabs(desired_angular_vel) > max_allowed_w)
    desired_angular_vel = std::copysign(max_allowed_w, desired_angular_vel);

  // 计算角速度增量
  double angular_increment = desired_angular_vel - current_angular_vel;

  // 根据当前和期望角速度差值动态调整最大角加速度
  double effective_max_w_inc = max_w_inc_;
  if (std::fabs(angular_increment) > max_w_inc_ * 2.0)
  {
    // 当需要大的角速度变化时，允许更大的角加速度
    effective_max_w_inc = max_w_inc_ * 1.5;
  }
  else if (std::fabs(angular_increment) < max_w_inc_ * 0.5)
  {
    // 当需要小的角速度变化时，减小角加速度以获得更平滑的控制
    effective_max_w_inc = max_w_inc_ * 0.8;
  }

  // 限制角加速度
  if (std::fabs(angular_increment) > effective_max_w_inc)
    angular_increment = std::copysign(effective_max_w_inc, angular_increment);

  // 应用增量
  double command_angular_vel = current_angular_vel + angular_increment;

  // 再次确保不超过最大值
  if (std::fabs(command_angular_vel) > max_allowed_w)
    command_angular_vel = std::copysign(max_allowed_w, command_angular_vel);
  else if (std::fabs(command_angular_vel) < min_w_ && desired_angular_vel != 0.0)
    command_angular_vel = std::copysign(min_w_, command_angular_vel);

  return command_angular_vel;
}

double RPPController::dphi(geometry_msgs::msg::PointStamped lookahead_pt, geometry_msgs::msg::PoseStamped robot_pose)
{
  // 计算机器人到前瞻点的角度差
  double global_angle =
      atan2(lookahead_pt.point.y - robot_pose.pose.position.y, lookahead_pt.point.x - robot_pose.pose.position.x);

  double robot_angle = tf2::getYaw(robot_pose.pose.orientation);

  return regularizeAngle(global_angle - robot_angle);
}
double RPPController::applyCurvatureConstraint(const double raw_linear_vel, const double curvature)
{
  if (std::fabs(curvature) < 1e-10)
    return raw_linear_vel;  // 避免除以零

  // 计算半径（曲率的倒数）
  double radius = 1.0 / std::fabs(curvature);

  // 更精确的速度曲率关系
  double constrained_vel = raw_linear_vel;

  if (radius < regulated_min_radius_)
  {
    // 使用二次关系而非线性关系
    double ratio = (radius / regulated_min_radius_);
    ratio = ratio * ratio;  // 二次关系使减速更加平滑

    // 确保不低于最小速度
    constrained_vel = std::max(min_v_, raw_linear_vel * ratio);

    RCLCPP_DEBUG(get_logger(), "应用曲率约束: 半径 %.2f < %.2f, 原速度 %.2f, 新速度 %.2f", radius,
                 regulated_min_radius_, raw_linear_vel, constrained_vel);
  }

  // 当曲率极大时进一步限制速度
  if (std::fabs(curvature) > 5.0)
  {
    constrained_vel = std::min(constrained_vel, min_v_ * 1.2);
  }

  return constrained_vel;
}

double RPPController::applyApproachConstraint(const double raw_linear_vel, geometry_msgs::msg::PoseStamped robot_pose,
                                              const std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan)
{
  if (pruned_plan.empty())
    return raw_linear_vel;

  // 计算剩余路径长度
  double remaining_length = 0.0;
  for (size_t i = 0; i < pruned_plan.size() - 1; ++i)
  {
    remaining_length += std::hypot(pruned_plan[i + 1].pose.position.x - pruned_plan[i].pose.position.x,
                                   pruned_plan[i + 1].pose.position.y - pruned_plan[i].pose.position.y);
  }

  // 应用接近约束
  if (remaining_length < approach_dist_)
  {
    // 计算到终点的直线距离
    double distance_to_end = std::hypot(pruned_plan.back().pose.position.x - robot_pose.pose.position.x,
                                        pruned_plan.back().pose.position.y - robot_pose.pose.position.y);

    // 按比例降低速度，但不低于最小速度
    double ratio = distance_to_end / approach_dist_;
    RCLCPP_DEBUG(get_logger(), "应用接近约束: 剩余长度 %.2f < %.2f, 比例 %.2f", remaining_length, approach_dist_,
                 ratio);

    return std::max(approach_min_v_, raw_linear_vel * ratio);
  }

  return raw_linear_vel;
}

bool RPPController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& robot_pose,
                                            const geometry_msgs::msg::Twist& current_velocity,
                                            geometry_msgs::msg::TwistStamped& cmd_vel)
{
  if (!initialized_ || global_plan_.poses.empty())
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化或路径为空");
    return false;
  }

  // 先检查是否已到达目标
  if (goal_reached_)
  {
    // 如果目标已经达到，立即停止机器人并返回
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    cmd_vel.header.stamp = this->now();
    cmd_vel.header.frame_id = "base_link";

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "目标已达到，保持停止状态");
    return true;
  }

  static int waiting_count = 0;
  if (waiting_)
  {
    if (waiting_count++ > 10)
    {
      waiting_count = 0;
      waiting_ = false;
    }

    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    cmd_vel.header.stamp = this->now();
    cmd_vel.header.frame_id = "base_link";

    return true;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header = robot_pose.header;
  current_pose.pose.orientation = robot_pose.pose.orientation;

  double filtered_x = h_x_filter.filter(robot_pose.pose.position.x);
  double filtered_y = h_y_filter.filter(robot_pose.pose.position.y);

  double smoothed_robot_x = filtered_x;
  double smoothed_robot_y = filtered_y;

  // 使用 Savitzky-Golay 滤波器平滑位置数据
  smoothed_robot_x = sg_x_filter_.filter(smoothed_robot_x);
  smoothed_robot_y = sg_y_filter_.filter(smoothed_robot_y);

  if (pos_use_biquad_cascade_filter_)
  {
    smoothed_robot_x = pos_x_filter_.filter(smoothed_robot_x);
    smoothed_robot_y = pos_y_filter_.filter(smoothed_robot_y);
  }

  current_pose.pose.position.x = smoothed_robot_x;
  current_pose.pose.position.y = smoothed_robot_y;

  current_velocity_ = current_velocity;

  // 设置默认安全速度命令
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = "base_link";

  static bool filter_reset = false;
  try
  {
    // 计时开始
    auto start_time = this->now();

    // 增加位置有效性检查
    if (!std::isfinite(current_pose.pose.position.x) || !std::isfinite(current_pose.pose.position.y))
    {
      RCLCPP_ERROR(get_logger(), "接收到无效的机器人位置: (%f, %f)", current_pose.pose.position.x,
                   current_pose.pose.position.y);
      return false;
    }

    // ==================== 圆形路径的航向预对准处理 ====================
    if ((is_circle_path || back_follow_) && need_yaw_prealign_ && !yaw_prealign_done_)
    {
      // 执行航向预对准
      bool alignment_complete = performYawPrealignment(current_pose, target_yaw_, cmd_vel);

      if (alignment_complete)
      {
        yaw_prealign_done_ = true;
        need_yaw_prealign_ = false;
        waiting_ = true;
        RCLCPP_INFO(get_logger(), "圆形路径航向预对准完成，开始路径跟随");
      }

      return true;  // 对准期间直接返回，不执行路径跟随
    }

    if (back_follow_)
    {
      // 修改航向角：加π并限制范围到[-π, π]
      double current_yaw = tf2::getYaw(current_pose.pose.orientation);
      double modified_yaw = current_yaw + M_PI;
      modified_yaw = atan2(sin(modified_yaw), cos(modified_yaw));  // 规范化到[-π, π]

      tf2::Quaternion q;
      q.setRPY(0, 0, modified_yaw);
      current_pose.pose.orientation = tf2::toMsg(q);
    }

    // 检查是否已到达目标
    auto goal_pose = global_plan_.poses.back();
    double distance_to_goal = std::hypot(current_pose.pose.position.x - goal_pose.pose.position.x,
                                         current_pose.pose.position.y - goal_pose.pose.position.y);

    remaining_distance_ = distance_to_goal;
    traversed_distance_ = path_length_ - remaining_distance_;

    if (is_circle_path)
    {
      // 收集机器人中心位置用于半径计算
      if (collect_positions_for_circle_)
      {
        collectPositionForRadiusCalculation(current_pose);

        // 同时收集喷墨口位置（与机器人中心使用相同的收集条件）
        if (!offset_points_.empty())
        {
          collectOffsetPointsPositions(current_pose);
        }
      }

      // 圆形路径，机器人旋转角度累加到指定值则判断为到达
      // 1. 获取当前航向角
      tf2::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                        current_pose.pose.orientation.z, current_pose.pose.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      // 2. 计算角度变化
      if (!last_yaw_initialized_)
      {
        // 首次进入，初始化上一次的航向角
        last_yaw_ = yaw;
        last_yaw_initialized_ = true;
        accumulated_angle_ = 0.0;
        RCLCPP_INFO(get_logger(), "圆形路径跟踪开始，初始航向角: %.2f", yaw);
      }
      else
      {
        // 计算角度差（考虑角度跳变）
        double delta_yaw = yaw - last_yaw_;

        // 处理角度跳变（从+π到-π或从-π到+π）
        if (delta_yaw > M_PI)
        {
          delta_yaw -= 2.0 * M_PI;
        }
        else if (delta_yaw < -M_PI)
        {
          delta_yaw += 2.0 * M_PI;
        }

        // 累加角度变化（只计算逆时针方向，对应左转）
        if (delta_yaw > 0)
        {
          accumulated_angle_ += delta_yaw;
        }
        else
        {
          // 如果是顺时针（右转），可能是路径跟踪偏差，不计入累计角度
          // 或者可以根据需要将其视为负贡献
          // accumulated_angle_ += delta_yaw;  // 取消注释如果需要计算右转
        }

        // 更新上一次航向角
        last_yaw_ = yaw;

        // 判断是否达到目标
        if (accumulated_angle_ >= (circle_total_angle - 0.5 * M_PI))
        {
          // 先计算并保存机器人中心的半径
          if (collect_positions_for_circle_ && collected_positions_.size() >= 10)
          {
            if (fitCircleToPositions(collected_positions_, robot_center_fitted_x_, robot_center_fitted_y_,
                                     robot_center_fitted_radius_))
            {
              robot_center_fit_success_ = true;

              double radius_error = std::abs(robot_center_fitted_radius_ - target_radius_);
              double center_error = std::sqrt(std::pow(robot_center_fitted_x_ - target_center_x_, 2) +
                                              std::pow(robot_center_fitted_y_ - target_center_y_, 2));

              RCLCPP_INFO(get_logger(), "机器人中心圆形拟合完成:");
              RCLCPP_INFO(get_logger(), "  目标圆心: (%.4f, %.4f), 目标半径: %.4f m", target_center_x_,
                          target_center_y_, target_radius_);
              RCLCPP_INFO(get_logger(), "  拟合圆心: (%.4f, %.4f), 拟合半径: %.4f m", robot_center_fitted_x_,
                          robot_center_fitted_y_, robot_center_fitted_radius_);
              RCLCPP_INFO(get_logger(), "  半径误差: %.4f m, 圆心误差: %.4f m", radius_error, center_error);
              RCLCPP_INFO(get_logger(), "  收集点数: %zu", collected_positions_.size());
            }
            else
            {
              robot_center_fit_success_ = false;
              RCLCPP_ERROR(get_logger(), "机器人中心圆形拟合失败");
            }
          }
          else
          {
            robot_center_fit_success_ = false;
          }

          // 接着计算并保存喷墨口的半径
          if (!offset_points_.empty())
          {
            calculateAndSaveOffsetPointsRadius();
          }

          // 重置收集标志
          collect_positions_for_circle_ = false;
          collected_positions_.clear();

          // 目标已达到，设置标志并停止机器人
          goal_reached_ = true;
          cmd_vel.twist.linear.x = 0.0;
          cmd_vel.twist.angular.z = 0.0;
          cmd_vel.header.stamp = this->now();
          cmd_vel.header.frame_id = "base_link";

          RCLCPP_INFO(get_logger(), "圆形路径完成 - 累计角度: %.4f rad (%.2f°)", accumulated_angle_,
                      accumulated_angle_ * 180.0 / M_PI);

          // 重置角度计算相关变量
          last_yaw_initialized_ = false;
          accumulated_angle_ = 0.0;

          // 输出性能统计
          if (error_count_ > 0)
          {
            avg_error_ = error_sum_ / error_count_;
            RCLCPP_INFO(get_logger(), "跟踪性能统计 - 最大误差: %.4f m, 平均误差: %.4f m", max_error_, avg_error_);
          }

          return true;
        }
      }
    }
    else
    {
      // 修改目标判断逻辑，不再进行最终旋转调整
      if (distance_to_goal < goal_dist_tol_)
      {
        // 目标已达到，设置标志并停止机器人
        goal_reached_ = true;
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        cmd_vel.header.stamp = this->now();
        cmd_vel.header.frame_id = "base_link";

        RCLCPP_INFO(get_logger(), "目标已达到 - 最终误差: %.4fm", remaining_distance_);

        // 输出性能统计
        if (error_count_ > 0)
        {
          avg_error_ = error_sum_ / error_count_;
          RCLCPP_INFO(get_logger(), "跟踪性能统计 - 最大误差: %.4f m, 平均误差: %.4f m", max_error_, avg_error_);
        }

        return true;
      }
    }

    // 裁剪全局路径，移除已经通过的点
    std::vector<geometry_msgs::msg::PoseStamped> pruned_plan;
    pruneGlobalPlan(current_pose, global_plan_, pruned_plan);

    if (pruned_plan.empty())
    {
      RCLCPP_ERROR(get_logger(), "裁剪后的路径为空");
      return false;
    }

    // 验证裁剪后的路径点
    bool has_invalid_points = false;
    for (size_t i = 0; i < pruned_plan.size(); ++i)
    {
      if (!std::isfinite(pruned_plan[i].pose.position.x) || !std::isfinite(pruned_plan[i].pose.position.y))
      {
        RCLCPP_ERROR(get_logger(), "裁剪后的路径包含无效点 #%zu: (%f, %f)", i, pruned_plan[i].pose.position.x,
                     pruned_plan[i].pose.position.y);
        has_invalid_points = true;
      }
    }

    if (has_invalid_points)
    {
      // 过滤无效点
      std::vector<geometry_msgs::msg::PoseStamped> valid_pruned_plan;
      for (const auto& pose : pruned_plan)
      {
        if (std::isfinite(pose.pose.position.x) && std::isfinite(pose.pose.position.y))
        {
          valid_pruned_plan.push_back(pose);
        }
      }

      if (valid_pruned_plan.empty())
      {
        RCLCPP_ERROR(get_logger(), "过滤无效点后路径为空");
        return false;
      }

      pruned_plan = valid_pruned_plan;
    }

    // 计算前瞻距离 - 使用改进的动态前瞻距离计算
    double lookahead_distance = getLookAheadDistance(current_velocity.linear.x);

    // 获取前瞻点
    geometry_msgs::msg::PoseStamped lookahead_pose;
    try
    {
      lookahead_pose = getLookAheadPoint(lookahead_distance, pruned_plan, true);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "获取前瞻点时发生异常: %s", e.what());

      // 使用备选前瞻点
      if (!pruned_plan.empty())
      {
        lookahead_pose = pruned_plan[std::min(size_t(1), pruned_plan.size() - 1)];
        RCLCPP_WARN(get_logger(), "使用备选前瞻点: (%f, %f)", lookahead_pose.pose.position.x,
                    lookahead_pose.pose.position.y);
      }
      else
      {
        return false;
      }
    }

    // 检查前瞻点是否有效
    if (!std::isfinite(lookahead_pose.pose.position.x) || !std::isfinite(lookahead_pose.pose.position.y))
    {
      RCLCPP_ERROR(get_logger(), "计算出的前瞻点无效: (%f, %f)", lookahead_pose.pose.position.x,
                   lookahead_pose.pose.position.y);

      // 使用安全的前瞻点
      if (!pruned_plan.empty())
      {
        lookahead_pose = pruned_plan[std::min(size_t(1), pruned_plan.size() - 1)];
        RCLCPP_WARN(get_logger(), "使用备选前瞻点: (%f, %f)", lookahead_pose.pose.position.x,
                    lookahead_pose.pose.position.y);
      }
      else
      {
        return false;
      }
    }

    // 转换为PointStamped用于计算
    geometry_msgs::msg::PointStamped lookahead_pt;
    lookahead_pt.header = lookahead_pose.header;
    lookahead_pt.point = lookahead_pose.pose.position;

    // 计算机器人与前瞻点之间的角度差
    double angle_to_lookahead = 0.0;
    try
    {
      angle_to_lookahead = dphi(lookahead_pt, current_pose);

      // 后退模式下需要调整角度
      // if (back_follow_ && !is_circle_path)
      // {
      //   // 后退时，需要将角度反转180度
      //   angle_to_lookahead = regularizeAngle(angle_to_lookahead + M_PI);
      // }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "计算角度差时发生异常: %s", e.what());
      angle_to_lookahead = 0.0;  // 使用安全默认值
    }

    // 检查角度是否有效
    if (!std::isfinite(angle_to_lookahead))
    {
      RCLCPP_ERROR(get_logger(), "计算出的角度无效: %f", angle_to_lookahead);
      angle_to_lookahead = 0.0;  // 使用安全默认值
    }

    RCLCPP_DEBUG(get_logger(), "到前瞻点的角度: %.2f rad", angle_to_lookahead);

    // 计算曲率 (使用Pure Pursuit公式)
    current_curvature_ = 0.0;
    if (std::abs(lookahead_distance) > 1e-6)
    {
      current_curvature_ = 2.0 * sin(angle_to_lookahead) / lookahead_distance;
    }
    else
    {
      RCLCPP_WARN(get_logger(), "前瞻距离过小: %f，使用默认曲率", lookahead_distance);
    }

    // 检查曲率是否有效
    if (!std::isfinite(current_curvature_))
    {
      RCLCPP_ERROR(get_logger(), "计算出的曲率无效: %f", current_curvature_);
      current_curvature_ = 0.0;  // 使用安全默认值
    }

    RCLCPP_DEBUG(get_logger(), "曲率: %.4f", current_curvature_);

    // 计算当前误差 (横向误差)
    current_lateral_error_ = 0.0;
    if (std::abs(lookahead_distance) > 1e-6 && std::isfinite(angle_to_lookahead))
    {
      current_lateral_error_ = lookahead_distance * sin(angle_to_lookahead);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "无法计算横向误差，使用默认值0");
    }

    // 检查横向误差是否有效
    if (!std::isfinite(current_lateral_error_))
    {
      RCLCPP_ERROR(get_logger(), "计算出的横向误差无效: %f", current_lateral_error_);
      current_lateral_error_ = 0.0;  // 使用安全默认值
    }

    // 更新误差统计
    if (std::isfinite(current_lateral_error_))
    {
      error_sum_ += std::fabs(current_lateral_error_);
      error_count_++;
      if (std::fabs(current_lateral_error_) > max_error_)
      {
        max_error_ = std::fabs(current_lateral_error_);
      }
      avg_error_ = error_sum_ / error_count_;
    }

    // 应用曲率约束 - 曲率大时降低速度以提高精度
    double desired_velocity = linear_speed_;
    try
    {
      desired_velocity = applyCurvatureConstraint(desired_velocity, current_curvature_);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "应用曲率约束时发生异常: %s", e.what());
      // 使用默认值
      desired_velocity = linear_speed_ * 0.5;  // 降低为基准速度的一半
    }

    // 计算角速度 (Pure Pursuit控制律)
    double desired_angular_velocity = 0.0;

    if (std::isfinite(desired_velocity) && std::isfinite(current_curvature_))
    {
      // 基础Pure Pursuit控制律计算
      double pp_angular_velocity = desired_velocity * current_curvature_;

      desired_angular_velocity = pp_angular_velocity;

      if (is_circle_path)
      {
        // 圆形路径处理
        if (accumulated_angle_ >= ((start_deviation_factor_ - 0.1) * M_PI))
        {
          // 限制角速度变化幅度
          double angular_velocity_delta = desired_angular_velocity - baseline_angular_velocity_for_circle_;
          double error_ratio = 0.05;  // 允许5%的偏差
          if (accumulated_angle_ < (start_deviation_factor_ * M_PI))
          {
            error_ratio = deviation_rate_;  // 允许偏差百分比
          }
          if (accumulated_angle_ > (circle_total_angle - end_deviation_factor_ * M_PI))
          {
            error_ratio = deviation_rate_;  // 允许偏差百分比
          }

          double max_angular_delta = baseline_angular_velocity_for_circle_ * error_ratio;
          if (std::abs(angular_velocity_delta) > max_angular_delta)
          {
            angular_velocity_delta = std::copysign(max_angular_delta, angular_velocity_delta);
          }
          desired_angular_velocity = baseline_angular_velocity_for_circle_ + angular_velocity_delta;
          desired_angular_velocity = smoothAngularVelocity(current_velocity.angular.z, desired_angular_velocity,
                                                           lookahead_distance, angle_to_lookahead, d_t_, filter_reset);

          filter_reset = false;
        }
        else
        {
          desired_angular_velocity = smoothAngularVelocity(current_velocity.angular.z, pp_angular_velocity,
                                                           lookahead_distance, angle_to_lookahead, d_t_, false);
          filter_reset = true;
        }
      }
      else
      {
        // 曲线路径处理
        // desired_angular_velocity = smoothAngularVelocity(current_velocity.angular.z, pp_angular_velocity,
        //                                                  lookahead_distance, angle_to_lookahead, d_t_, false);
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "计算角速度的输入无效: 线速度=%f, 曲率=%f", desired_velocity, current_curvature_);
      desired_angular_velocity = 0.0;  // 使用安全默认值
    }

    // RCLCPP_INFO(get_logger(), "期望角速度: %.2f rad/s", desired_angular_velocity);

    // 检查角速度是否有效
    if (!std::isfinite(desired_angular_velocity))
    {
      RCLCPP_ERROR(get_logger(), "计算出的期望角速度无效: %f", desired_angular_velocity);
      desired_angular_velocity = 0.0;  // 使用安全默认值
    }

    // 应用正则化和平滑处理
    try
    {
      cmd_vel.twist.linear.x = linearRegularization(current_velocity.linear.x, desired_velocity);
      desired_velocity_ = desired_velocity;

      cmd_vel.twist.angular.z = desired_angular_velocity;

      // 后退模式下反转线速度和角速度
      if (back_follow_ && !is_circle_path)
      {
        cmd_vel.twist.linear.x = -cmd_vel.twist.linear.x;
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "角速度平滑处理时发生异常: %s", e.what());
      // 使用安全值
      cmd_vel.twist.angular.z = 0.0;
    }

    // 确保输出的命令是有效值
    if (!std::isfinite(cmd_vel.twist.linear.x))
    {
      RCLCPP_ERROR(get_logger(), "输出线速度无效: %f, 使用安全默认值", cmd_vel.twist.linear.x);
      cmd_vel.twist.linear.x = min_v_;
    }

    if (!std::isfinite(cmd_vel.twist.angular.z))
    {
      RCLCPP_ERROR(get_logger(), "输出角速度无效: %f, 使用安全默认值", cmd_vel.twist.angular.z);
      cmd_vel.twist.angular.z = 0.0;
    }

    cmd_vel.header.stamp = this->now();
    cmd_vel.header.frame_id = "base_link";

    // 输出调试信息 (减少日志频率以避免性能影响)
    static int debug_counter = 0;
    if (++debug_counter % 10 == 0)
    {  // 每10次迭代输出一次
      // 记录详细调试信息
      std::stringstream debug_ss;
      debug_ss << "路径跟随详情:" << std::endl
               << "  当前位置: (" << std::fixed << std::setprecision(3) << current_pose.pose.position.x << ", "
               << current_pose.pose.position.y << ")" << std::endl
               << "  目标距离: " << remaining_distance_ << "m" << std::endl
               << "  横向误差: " << current_lateral_error_ << "m" << std::endl
               << "  前瞻距离: " << lookahead_distance << "m" << std::endl
               << "  前瞻点: (" << lookahead_pose.pose.position.x << ", " << lookahead_pose.pose.position.y << ")"
               << std::endl
               << "  曲率: " << current_curvature_ << std::endl
               << "  期望速度: " << desired_velocity << "m/s" << std::endl
               << "  输出线速度: " << cmd_vel.twist.linear.x << "m/s" << std::endl
               << "  输出角速度: " << cmd_vel.twist.angular.z << "rad/s" << std::endl
               << "  平均误差: " << avg_error_ << "m" << std::endl
               << "  最大误差: " << max_error_ << "m" << std::endl
               << "  后退模式: " << (back_follow_ ? "启用" : "禁用");

      // RCLCPP_INFO(get_logger(), "%s", debug_ss.str().c_str());
    }

    // 在栅格图上更新机器人位置和前瞻点（如果启用）
    if (enable_grid_map_)
    {
      // 每1s更新一次栅格图，避免过于频繁的更新
      auto current_time = this->now();
      if ((current_time - last_grid_update_time_).seconds() > 1.0)
      {
        // 绘制机器人
        drawRobotOnGrid(current_pose);

        // 绘制前瞻点
        if (std::isfinite(lookahead_pose.pose.position.x) && std::isfinite(lookahead_pose.pose.position.y))
        {
          drawLookaheadPointOnGrid(lookahead_pose.pose.position);
        }

        // 保存栅格图
        saveGridMap();
        last_grid_update_time_ = current_time;
      }
    }

    // 计时结束，计算控制循环耗时
    auto end_time = this->now();
    auto duration = end_time - start_time;

    // 如果控制循环耗时超过预期，输出警告
    if (duration.seconds() > d_t_ * 0.9)
    {
      RCLCPP_WARN(get_logger(), "控制循环耗时过长: %.3f ms", duration.seconds() * 1000.0);
    }

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "计算速度命令时发生异常: %s", e.what());
    return false;
  }
  catch (...)
  {
    RCLCPP_ERROR(get_logger(), "计算速度命令时发生未知异常");
    return false;
  }
}

void RPPController::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& current_pose,
                                    const nav_msgs::msg::Path& global_plan,
                                    std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan)
{
  pruned_plan.clear();

  if (global_plan.poses.empty())
    return;

  // 找到最近的路径点
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < global_plan.poses.size(); ++i)
  {
    double dist = std::hypot(global_plan.poses[i].pose.position.x - current_pose.pose.position.x,
                             global_plan.poses[i].pose.position.y - current_pose.pose.position.y);

    if (dist < min_dist)
    {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // 添加最近点以及后面的所有点到裁剪后的路径
  pruned_plan.insert(pruned_plan.end(), global_plan.poses.begin() + closest_idx, global_plan.poses.end());

  RCLCPP_DEBUG(get_logger(), "裁剪路径：原始点数 %zu, 裁剪后点数 %zu, 最近点索引 %zu", global_plan.poses.size(),
               pruned_plan.size(), closest_idx);
}

geometry_msgs::msg::PoseStamped RPPController::getLookAheadPoint(
    const double& lookahead_dist, const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
    bool interpolate_after_goal)
{
  if (transformed_plan.empty())
  {
    RCLCPP_ERROR(get_logger(), "接收到空路径，无法计算前瞻点");
    return geometry_msgs::msg::PoseStamped();
  }

  // 使用改进的前瞻点搜索算法
  const auto& robot_pose = transformed_plan.front();

  // 处理退化情况
  if (transformed_plan.size() == 1)
  {
    return transformed_plan[0];
  }

  // 使用投影距离而非直线距离进行前瞻点搜索
  double accumulated_dist = 0.0;
  size_t i = 0;

  // 计算线段上累积距离，直到找到前瞻距离位置
  for (i = 0; i < transformed_plan.size() - 1; ++i)
  {
    // 计算当前线段长度
    double segment_length = std::hypot(transformed_plan[i + 1].pose.position.x - transformed_plan[i].pose.position.x,
                                       transformed_plan[i + 1].pose.position.y - transformed_plan[i].pose.position.y);

    // 如果加上这段距离超过了前瞻距离，则在当前线段上找前瞻点
    if (accumulated_dist + segment_length >= lookahead_dist)
    {
      double remaining_dist = lookahead_dist - accumulated_dist;
      double ratio = remaining_dist / segment_length;

      // 线性插值计算前瞻点
      geometry_msgs::msg::PoseStamped lookahead_pose;
      lookahead_pose.header = transformed_plan[i].header;
      lookahead_pose.pose.position.x =
          transformed_plan[i].pose.position.x +
          ratio * (transformed_plan[i + 1].pose.position.x - transformed_plan[i].pose.position.x);
      lookahead_pose.pose.position.y =
          transformed_plan[i].pose.position.y +
          ratio * (transformed_plan[i + 1].pose.position.y - transformed_plan[i].pose.position.y);

      // 计算朝向
      double yaw = atan2(transformed_plan[i + 1].pose.position.y - transformed_plan[i].pose.position.y,
                         transformed_plan[i + 1].pose.position.x - transformed_plan[i].pose.position.x);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      lookahead_pose.pose.orientation = tf2::toMsg(q);

      RCLCPP_DEBUG(get_logger(), "找到前瞻点: (%.3f, %.3f)，在路径点%zu和%zu之间", lookahead_pose.pose.position.x,
                   lookahead_pose.pose.position.y, i, i + 1);
      return lookahead_pose;
    }

    accumulated_dist += segment_length;
  }

  // 如果循环结束仍未找到，返回最后一个点
  RCLCPP_DEBUG(get_logger(), "前瞻距离超出路径长度，使用最后一个点");
  return transformed_plan.back();
}

// 执行航向预对准
bool RPPController::performYawPrealignment(const geometry_msgs::msg::PoseStamped& current_pose, double target_yaw,
                                           geometry_msgs::msg::TwistStamped& cmd_vel)
{
  // 获取当前航向角
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);

  // 计算角度差
  double yaw_error = angles::shortest_angular_distance(current_yaw, target_yaw);

  // 设置角度对准的容忍度（可以设为参数）
  double yaw_tolerance = 0.05;  // 约3度

  if (std::abs(yaw_error) < yaw_tolerance)
  {
    // 航向对准完成
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    cmd_vel.header.stamp = this->now();
    cmd_vel.header.frame_id = "base_link";

    RCLCPP_INFO(get_logger(), "航向预对准完成！角度误差: %.3f rad (%.1f°)", yaw_error, yaw_error * 180.0 / M_PI);
    return true;
  }

  double angular_vel = calculateRotationVelocity(yaw_error);

  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = angular_vel;
  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = "base_link";

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                       "航向预对准中... 目标: %.3f, 当前: %.3f, 误差: %.3f, 角速度: %.3f", target_yaw, current_yaw,
                       yaw_error, angular_vel);

  return false;  // 对准尚未完成
}

double RPPController::calculateRotationVelocity(const double& angle_diff)
{
  // 使用sigmoid函数计算旋转因子
  double factor = 1.0 / (1.0 + std::exp(-1.0 * std::abs(angle_diff)));

  // 在接近目标时使用余弦曲线平滑减速
  if (std::abs(angle_diff) < 0.5)
  {
    double cosine_factor = 0.5 * (1.0 - std::cos(M_PI * std::abs(angle_diff) / 0.5));
    factor *= cosine_factor;
  }

  // 计算旋转速度
  double rot_vel = 1.0 * factor;
  rot_vel = std::max(rot_vel, 0.2);

  // 根据角度差的符号决定旋转方向
  return (angle_diff > 0.0) ? rot_vel : -rot_vel;
}

double RPPController::smoothAngularVelocity(double current_angular_vel, double desired_angular_vel,
                                            double lookahead_dist, double angle_to_path, double dt, bool is_reset)
{
  if (is_reset)
  {
    angular_vel_history_.clear();
    second_order_filter_.reset();
    previous_angular_vel_ = 0.0;
  }

  // 添加到历史记录
  angular_vel_history_.push_back(desired_angular_vel);
  if (angular_vel_history_.size() > angular_vel_history_size_)
  {
    angular_vel_history_.pop_front();
  }

  double smoothed_angular_vel = desired_angular_vel;

  if (smoothing_type_ == "lowpass")
  {
    // 低通滤波器
    smoothed_angular_vel = lowpass_angular_vel_filter_gain_ * desired_angular_vel +
                           (1 - lowpass_angular_vel_filter_gain_) * previous_angular_vel_;
  }

  if (smoothing_type_ == "movingAverage")
  {
    for (double vel : angular_vel_history_)
    {
      smoothed_angular_vel += vel;
    }
    smoothed_angular_vel /= angular_vel_history_.size();
  }

  // 使用二阶平滑器
  smoothed_angular_vel = second_order_filter_.filter(smoothed_angular_vel, dt);

  if (angle_use_biquad_cascade_filter_)
  {
    smoothed_angular_vel = angle_vel_filter_.filter(smoothed_angular_vel);
  }

  // 更新历史数据
  previous_angular_vel_ = smoothed_angular_vel;
  angle_to_path_prev_ = angle_to_path;
  lookahead_dist_prev_ = lookahead_dist;

  return smoothed_angular_vel;
}

void RPPController::robotToGlobalCoordinate(const geometry_msgs::msg::PoseStamped& robot_pose, double x_offset,
                                            double y_offset, double& global_x, double& global_y)
{
  // 创建机器人坐标系到全局坐标系的变换矩阵
  tf2::Transform robot_to_global_transform;

  // 设置平移
  robot_to_global_transform.setOrigin(
      tf2::Vector3(robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z));

  // 设置旋转
  tf2::Quaternion robot_quaternion;
  tf2::fromMsg(robot_pose.pose.orientation, robot_quaternion);
  robot_to_global_transform.setRotation(robot_quaternion);

  // 创建机器人坐标系中的偏置点
  tf2::Vector3 offset_point_robot(x_offset, y_offset, 0.0);

  // 将偏置点变换到全局坐标系
  tf2::Vector3 offset_point_global = robot_to_global_transform * offset_point_robot;

  // 输出全局坐标
  global_x = offset_point_global.x();
  global_y = offset_point_global.y();

  RCLCPP_DEBUG(get_logger(), "坐标变换: 机器人坐标(%.3f, %.3f) -> 全局坐标(%.3f, %.3f)", x_offset, y_offset, global_x,
               global_y);
}

void RPPController::collectPositionForRadiusCalculation(const geometry_msgs::msg::PoseStamped& pose)
{
  auto current_time = this->now();

  // 按时间间隔收集位置，避免过于密集
  if ((current_time - last_position_collect_time_).seconds() >= position_collect_interval_)
  {
    collected_positions_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    last_position_collect_time_ = current_time;

    // 限制收集的点数，避免内存过度使用
    if (collected_positions_.size() > 5000)
    {
      // 保留最新的2500个点
      collected_positions_.erase(collected_positions_.begin(), collected_positions_.begin() + 2500);
    }

    RCLCPP_DEBUG(get_logger(), "收集机器人中心位置: (%.4f, %.4f), 总数: %zu", pose.pose.position.x,
                 pose.pose.position.y, collected_positions_.size());
  }
}

void RPPController::collectOffsetPointsPositions(const geometry_msgs::msg::PoseStamped& robot_pose)
{
  // 与机器人中心使用相同的收集条件
  // 这个函数在collectPositionForRadiusCalculation之后立即调用

  for (auto& offset_point : offset_points_)
  {
    double global_x, global_y;

    // 将机器人坐标系偏置转换为全局坐标
    robotToGlobalCoordinate(robot_pose, offset_point.x_offset, offset_point.y_offset, global_x, global_y);

    // 收集全局坐标位置
    offset_point.positions.emplace_back(global_x, global_y);

    // 限制收集的点数，避免内存过度使用
    if (offset_point.positions.size() > 5000)
    {
      // 保留最新的2500个点
      offset_point.positions.erase(offset_point.positions.begin(), offset_point.positions.begin() + 2500);
    }

    RCLCPP_DEBUG(get_logger(), "收集喷墨口 %s 位置: (%.4f, %.4f), 总数: %zu", offset_point.name.c_str(), global_x,
                 global_y, offset_point.positions.size());
  }
}

bool RPPController::fitCircleToPositions(const std::vector<std::pair<double, double>>& positions, double& center_x,
                                         double& center_y, double& radius)
{
  if (positions.size() < 3)
  {
    RCLCPP_ERROR(get_logger(), "拟合圆形需要至少3个点，当前只有%zu个点", positions.size());
    return false;
  }

  try
  {
    // 使用代数法(Taubin方法)进行圆形拟合，该方法对噪声具有较好的鲁棒性
    size_t n = positions.size();

    // 计算质心
    double mean_x = 0.0, mean_y = 0.0;
    for (const auto& pos : positions)
    {
      mean_x += pos.first;
      mean_y += pos.second;
    }
    mean_x /= n;
    mean_y /= n;

    // 构建矩阵方程 A*[a, b, c]^T = B
    // 其中圆的方程为: x^2 + y^2 + a*x + b*y + c = 0
    std::vector<std::vector<double>> A(n, std::vector<double>(3));
    std::vector<double> B(n);

    for (size_t i = 0; i < n; ++i)
    {
      double x = positions[i].first - mean_x;  // 中心化坐标
      double y = positions[i].second - mean_y;

      A[i][0] = x;              // x项系数
      A[i][1] = y;              // y项系数
      A[i][2] = 1.0;            // 常数项系数
      B[i] = -(x * x + y * y);  // 右侧向量
    }

    // 使用最小二乘法求解: (A^T * A) * x = A^T * B
    std::vector<std::vector<double>> ATA(3, std::vector<double>(3, 0.0));
    std::vector<double> ATB(3, 0.0);

    // 计算A^T * A
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        for (size_t k = 0; k < n; ++k)
        {
          ATA[i][j] += A[k][i] * A[k][j];
        }
      }
    }

    // 计算A^T * B
    for (int i = 0; i < 3; ++i)
    {
      for (size_t k = 0; k < n; ++k)
      {
        ATB[i] += A[k][i] * B[k];
      }
    }

    // 高斯消元法求解3x3线性方程组
    std::vector<double> solution(3);

    // 前向消元
    for (int i = 0; i < 3; ++i)
    {
      // 寻找主元
      int max_row = i;
      for (int k = i + 1; k < 3; ++k)
      {
        if (std::abs(ATA[k][i]) > std::abs(ATA[max_row][i]))
        {
          max_row = k;
        }
      }

      // 交换行
      if (max_row != i)
      {
        std::swap(ATA[i], ATA[max_row]);
        std::swap(ATB[i], ATB[max_row]);
      }

      // 检查是否为奇异矩阵
      if (std::abs(ATA[i][i]) < 1e-12)
      {
        RCLCPP_ERROR(get_logger(), "矩阵奇异，无法求解");
        return false;
      }

      // 消元
      for (int k = i + 1; k < 3; ++k)
      {
        double factor = ATA[k][i] / ATA[i][i];
        for (int j = i; j < 3; ++j)
        {
          ATA[k][j] -= factor * ATA[i][j];
        }
        ATB[k] -= factor * ATB[i];
      }
    }

    // 回代求解
    for (int i = 2; i >= 0; --i)
    {
      solution[i] = ATB[i];
      for (int j = i + 1; j < 3; ++j)
      {
        solution[i] -= ATA[i][j] * solution[j];
      }
      solution[i] /= ATA[i][i];
    }

    // 计算圆心和半径
    double a = solution[0];
    double b = solution[1];
    double c = solution[2];

    center_x = -a / 2.0 + mean_x;  // 恢复到原始坐标系
    center_y = -b / 2.0 + mean_y;
    radius = std::sqrt((a * a + b * b) / 4.0 - c);

    // 验证结果的合理性
    if (radius <= 0 || radius > 100.0)  // 半径应该在合理范围内
    {
      RCLCPP_ERROR(get_logger(), "计算得到的半径不合理: %.4f", radius);
      return false;
    }

    // 计算拟合质量(均方根误差)
    double rms_error = 0.0;
    for (const auto& pos : positions)
    {
      double dist_to_center = std::sqrt(std::pow(pos.first - center_x, 2) + std::pow(pos.second - center_y, 2));
      double error = std::abs(dist_to_center - radius);
      rms_error += error * error;
    }
    rms_error = std::sqrt(rms_error / n);

    RCLCPP_DEBUG(get_logger(), "圆形拟合质量评估:");
    RCLCPP_DEBUG(get_logger(), "  均方根误差: %.6f m", rms_error);
    RCLCPP_DEBUG(get_logger(), "  拟合点数: %zu", n);

    // 如果拟合误差过大，认为拟合失败
    if (rms_error > 0.1)  // 10cm的误差阈值
    {
      RCLCPP_WARN(get_logger(), "拟合误差过大(%.4f m)，可能数据质量不佳", rms_error);
    }

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "圆形拟合过程中发生异常: %s", e.what());
    return false;
  }
}

void RPPController::calculateAndSaveOffsetPointsRadius()
{
  RCLCPP_INFO(get_logger(), "开始计算机器人中心和%zu个喷墨口的圆形拟合", offset_points_.size());

  // 首先计算机器人中心的圆形拟合
  if (collected_positions_.size() >= 10)
  {
    if (fitCircleToPositions(collected_positions_, robot_center_fitted_x_, robot_center_fitted_y_,
                             robot_center_fitted_radius_))
    {
      robot_center_fit_success_ = true;

      double radius_error = std::abs(robot_center_fitted_radius_ - target_radius_);
      double center_error = std::sqrt(std::pow(robot_center_fitted_x_ - target_center_x_, 2) +
                                      std::pow(robot_center_fitted_y_ - target_center_y_, 2));
      double radius_error_percentage = (radius_error / target_radius_) * 100.0;

      RCLCPP_INFO(get_logger(), "机器人中心圆形拟合完成:");
      RCLCPP_INFO(get_logger(), "  目标圆心: (%.4f, %.4f), 目标半径: %.4f m", target_center_x_, target_center_y_,
                  target_radius_);
      RCLCPP_INFO(get_logger(), "  拟合圆心: (%.4f, %.4f), 拟合半径: %.4f m", robot_center_fitted_x_,
                  robot_center_fitted_y_, robot_center_fitted_radius_);
      RCLCPP_INFO(get_logger(), "  半径误差: %.4f m (%.2f%%), 圆心误差: %.4f m", radius_error, radius_error_percentage,
                  center_error);
      RCLCPP_INFO(get_logger(), "  收集点数: %zu", collected_positions_.size());
    }
    else
    {
      robot_center_fit_success_ = false;
      RCLCPP_ERROR(get_logger(), "机器人中心圆形拟合失败");
    }
  }
  else
  {
    robot_center_fit_success_ = false;
    RCLCPP_WARN(get_logger(), "机器人中心收集的位置点不足，无法进行圆形拟合 (需要至少10个点，当前: %zu)",
                collected_positions_.size());
  }

  // 然后计算各个喷墨口的圆形拟合
  for (auto& offset_point : offset_points_)
  {
    if (offset_point.positions.size() < 10)
    {
      RCLCPP_WARN(get_logger(), "喷墨口 %s 收集的位置点不足，无法进行圆形拟合 (需要至少10个点，当前: %zu)",
                  offset_point.name.c_str(), offset_point.positions.size());
      offset_point.fit_success = false;
      continue;
    }

    if (fitCircleToPositions(offset_point.positions, offset_point.fitted_center_x, offset_point.fitted_center_y,
                             offset_point.fitted_radius))
    {
      offset_point.fit_success = true;

      double radius_error = std::abs(offset_point.fitted_radius - target_radius_);
      double center_error = std::sqrt(std::pow(offset_point.fitted_center_x - target_center_x_, 2) +
                                      std::pow(offset_point.fitted_center_y - target_center_y_, 2));
      double radius_error_percentage = (radius_error / target_radius_) * 100.0;

      RCLCPP_INFO(get_logger(), "喷墨口 %s 圆形拟合完成:", offset_point.name.c_str());
      RCLCPP_INFO(get_logger(), "  偏置量: (%.3f, %.3f) m", offset_point.x_offset, offset_point.y_offset);
      RCLCPP_INFO(get_logger(), "  目标圆心: (%.4f, %.4f), 目标半径: %.4f m", target_center_x_, target_center_y_,
                  target_radius_);
      RCLCPP_INFO(get_logger(), "  拟合圆心: (%.4f, %.4f), 拟合半径: %.4f m", offset_point.fitted_center_x,
                  offset_point.fitted_center_y, offset_point.fitted_radius);
      RCLCPP_INFO(get_logger(), "  半径误差: %.4f m (%.2f%%), 圆心误差: %.4f m", radius_error, radius_error_percentage,
                  center_error);
      RCLCPP_INFO(get_logger(), "  收集点数: %zu", offset_point.positions.size());

      // 保存详细数据到CSV
      saveDetailedCircleDataToCSV(offset_point);
    }
    else
    {
      offset_point.fit_success = false;
      RCLCPP_ERROR(get_logger(), "喷墨口 %s 圆形拟合失败", offset_point.name.c_str());
    }
  }
}

void RPPController::saveDetailedCircleDataToCSV(const OffsetPoint& offset_point)
{
  if (radius_log_file_path_.empty())
  {
    RCLCPP_WARN(get_logger(), "未设置半径日志文件路径，无法保存结果");
    return;
  }

  try
  {
    // 确保目录存在
    std::filesystem::path file_path(radius_log_file_path_);
    std::filesystem::create_directories(file_path.parent_path());

    // 检查文件是否存在，如果不存在则创建并写入表头
    bool file_exists = std::filesystem::exists(radius_log_file_path_);

    std::ofstream file(radius_log_file_path_, std::ios::app);
    if (!file.is_open())
    {
      RCLCPP_ERROR(get_logger(), "无法打开半径日志文件: %s", radius_log_file_path_.c_str());
      return;
    }

    // 如果是新文件，写入CSV表头
    if (!file_exists)
    {
      file << "timestamp,nozzle_name,x_offset_m,y_offset_m,"
           << "target_center_x_m,target_center_y_m,target_radius_m,"
           << "robot_center_fitted_x_m,robot_center_fitted_y_m,robot_center_fitted_radius_m,"
           << "nozzle_fitted_center_x_m,nozzle_fitted_center_y_m,nozzle_fitted_radius_m,"
           << "nozzle_radius_error_m,nozzle_radius_error_percent,nozzle_center_error_m,"
           << "robot_center_point_count,nozzle_point_count,robot_center_fit_success,nozzle_fit_success,"
           << "baseline_linear_speed_m_s,max_lookahead_distance_m\n";
    }

    // 计算喷墨口误差
    double nozzle_radius_error = 0.0;
    double nozzle_radius_error_percent = 0.0;
    double nozzle_center_error = 0.0;

    if (offset_point.fit_success)
    {
      nozzle_radius_error = std::abs(offset_point.fitted_radius - target_radius_);
      nozzle_radius_error_percent = (nozzle_radius_error / target_radius_) * 100.0;
      nozzle_center_error = std::sqrt(std::pow(offset_point.fitted_center_x - target_center_x_, 2) +
                                      std::pow(offset_point.fitted_center_y - target_center_y_, 2));
    }

    // 写入数据
    auto now = this->now();
    file << std::fixed << std::setprecision(6) << now.seconds() << "." << std::setfill('0') << std::setw(9)
         << now.nanoseconds() << "," << offset_point.name << "," << offset_point.x_offset << ","
         << offset_point.y_offset << "," << target_center_x_ << "," << target_center_y_ << "," << target_radius_ << ","
         << (robot_center_fit_success_ ? robot_center_fitted_x_ : 0.0) << ","
         << (robot_center_fit_success_ ? robot_center_fitted_y_ : 0.0) << ","
         << (robot_center_fit_success_ ? robot_center_fitted_radius_ : 0.0) << ","
         << (offset_point.fit_success ? offset_point.fitted_center_x : 0.0) << ","
         << (offset_point.fit_success ? offset_point.fitted_center_y : 0.0) << ","
         << (offset_point.fit_success ? offset_point.fitted_radius : 0.0) << "," << std::setprecision(6)
         << nozzle_radius_error << "," << std::setprecision(3) << nozzle_radius_error_percent << ","
         << std::setprecision(6) << nozzle_center_error << "," << collected_positions_.size() << ","
         << offset_point.positions.size() << "," << (robot_center_fit_success_ ? "true" : "false") << ","
         << (offset_point.fit_success ? "true" : "false") << "," << std::setprecision(4) << linear_speed_ << ","
         << max_lookahead_dist_ << "\n";

    file.close();

    RCLCPP_INFO(get_logger(), "喷墨口 %s 详细圆形数据已保存到: %s (基准线速度: %.3f m/s, 最大前瞻距离: %.3f m)",
                offset_point.name.c_str(), radius_log_file_path_.c_str(), linear_speed_, max_lookahead_dist_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "保存详细圆形数据时发生异常: %s", e.what());
  }
}

void RPPController::addOffsetPoint(double x_offset, double y_offset, const std::string& name)
{
  OffsetPoint offset_point;
  offset_point.x_offset = x_offset;
  offset_point.y_offset = y_offset;
  offset_point.name = name;
  offset_point.positions.clear();
  offset_points_.clear();
  offset_points_.push_back(offset_point);

  RCLCPP_INFO(get_logger(), "添加喷墨口: %s, 偏置量: (%.3f, %.3f)", name.c_str(), x_offset, y_offset);
}

void RPPController::setRadiusLogFilePath(const std::string& file_path)
{
  radius_log_file_path_ = file_path;
  RCLCPP_INFO(get_logger(), "设置半径日志文件路径为: %s", radius_log_file_path_.c_str());
}

void RPPController::clearOffsetPoints()
{
  offset_points_.clear();
  RCLCPP_INFO(get_logger(), "已清空所有喷墨口偏置点配置");
}

// 栅格图相关辅助函数
void RPPController::initializeGridMap(const nav_msgs::msg::Path& path)
{
  if (path.poses.empty())
  {
    RCLCPP_WARN(get_logger(), "无法初始化栅格图：路径为空");
    return;
  }

  // 计算路径的边界框
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto& pose : path.poses)
  {
    min_x = std::min(min_x, pose.pose.position.x);
    min_y = std::min(min_y, pose.pose.position.y);
    max_x = std::max(max_x, pose.pose.position.x);
    max_y = std::max(max_y, pose.pose.position.y);
  }

  // 为边界框添加边距
  double margin = 1.0;  // 1米的边距
  min_x -= margin;
  min_y -= margin;
  max_x += margin;
  max_y += margin;

  // 更新栅格图参数
  grid_width_ = max_x - min_x;
  grid_height_ = max_y - min_y;
  grid_origin_x_ = min_x;
  grid_origin_y_ = min_y;

  // 计算栅格图尺寸
  int width_pixels = static_cast<int>(grid_width_ / grid_resolution_);
  int height_pixels = static_cast<int>(grid_height_ / grid_resolution_);

  // 创建栅格图
  grid_map_ = cv::Mat(height_pixels, width_pixels, CV_8UC3, cv::Scalar(255, 255, 255));

  // 添加栅格线
  drawGridLines();

  // 记录初始化时间
  last_grid_update_time_ = this->now();

  // 在栅格图上绘制路径 - 使用更细的线条
  drawPathOnGrid(path, cv::Scalar(0, 0, 255), 1);  // 红色路径，线宽为1

  RCLCPP_INFO(get_logger(), "初始化栅格图 大小: %.2f x %.2f 米, 分辨率: %.3f 米/像素, 尺寸: %d x %d 像素", grid_width_,
              grid_height_, grid_resolution_, width_pixels, height_pixels);

  // 保存初始栅格图
  saveGridMap();
}

cv::Point RPPController::worldToGrid(double x, double y)
{
  int grid_x = static_cast<int>((x - grid_origin_x_) / grid_resolution_);
  int grid_y = static_cast<int>((y - grid_origin_y_) / grid_resolution_);

  // 垂直方向翻转，使y轴向上
  grid_y = grid_map_.rows - grid_y - 1;

  return cv::Point(grid_x, grid_y);
}

void RPPController::drawPathOnGrid(const nav_msgs::msg::Path& path, const cv::Scalar& color, int thickness)
{
  if (path.poses.empty() || grid_map_.empty())
    return;

  // 使用更细的线条绘制路径
  for (size_t i = 0; i < path.poses.size() - 1; ++i)
  {
    cv::Point pt1 = worldToGrid(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
    cv::Point pt2 = worldToGrid(path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y);

    // 确保点在图像范围内
    if (pt1.x >= 0 && pt1.x < grid_map_.cols && pt1.y >= 0 && pt1.y < grid_map_.rows && pt2.x >= 0 &&
        pt2.x < grid_map_.cols && pt2.y >= 0 && pt2.y < grid_map_.rows)
    {
      cv::line(grid_map_, pt1, pt2, color, thickness);
    }
  }

  // 标记起点和终点
  if (!path.poses.empty())
  {
    // 起点 - 绿色
    cv::Point start = worldToGrid(path.poses.front().pose.position.x, path.poses.front().pose.position.y);
    if (start.x >= 0 && start.x < grid_map_.cols && start.y >= 0 && start.y < grid_map_.rows)
    {
      cv::circle(grid_map_, start, 3, cv::Scalar(0, 255, 0), -1);
    }

    // 终点 - 蓝色
    cv::Point end = worldToGrid(path.poses.back().pose.position.x, path.poses.back().pose.position.y);
    if (end.x >= 0 && end.x < grid_map_.cols && end.y >= 0 && end.y < grid_map_.rows)
    {
      cv::circle(grid_map_, end, 3, cv::Scalar(255, 0, 0), -1);
    }
  }
}

void RPPController::drawRobotOnGrid(const geometry_msgs::msg::PoseStamped& pose)
{
  if (grid_map_.empty())
    return;

  // 获取机器人位置
  cv::Point robot_pos = worldToGrid(pose.pose.position.x, pose.pose.position.y);

  // 检查点是否在栅格图范围内
  if (robot_pos.x >= 0 && robot_pos.x < grid_map_.cols && robot_pos.y >= 0 && robot_pos.y < grid_map_.rows)
  {
    // 绘制机器人位置 - 黑色
    cv::circle(grid_map_, robot_pos, 1, cv::Scalar(0, 0, 0), -1);

    // 绘制机器人方向
    double yaw = tf2::getYaw(pose.pose.orientation);
    double arrow_length = 1.0;  // 箭头长度（像素）

    cv::Point arrow_end(robot_pos.x + static_cast<int>(arrow_length * cos(yaw)),
                        robot_pos.y - static_cast<int>(arrow_length * sin(yaw))  // 注意y轴方向已翻转
    );

    // 确保箭头端点在图像范围内
    if (arrow_end.x >= 0 && arrow_end.x < grid_map_.cols && arrow_end.y >= 0 && arrow_end.y < grid_map_.rows)
    {
      cv::arrowedLine(grid_map_, robot_pos, arrow_end, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, 0, 0.3);
    }
  }
}

void RPPController::saveGridMap()
{
  if (grid_map_.empty())
    return;

  // 同时保存最新的栅格图（覆盖先前的）
  try
  {
    std::string latest_filename = grid_map_path_ + "/grid_map_latest.png";
    cv::imwrite(latest_filename, grid_map_);
  }
  catch (const cv::Exception& e)
  {
    RCLCPP_ERROR(get_logger(), "保存最新栅格图失败: %s", e.what());
  }
}

void RPPController::drawGridLines()
{
  if (grid_map_.empty())
    return;

  // 设置栅格线的颜色和宽度
  cv::Scalar grid_color(220, 220, 220);  // 浅灰色
  int line_width = 1;

  // 每隔 n 个像素绘制一条栅格线
  int grid_stride = 5;  // 默认每5个像素绘制一条线

  // 根据分辨率调整栅格线间隔
  if (grid_resolution_ < 0.01)
  {  // 分辨率小于1厘米/像素
    grid_stride = 10;
  }
  else
  {
    grid_stride = std::max(1, static_cast<int>(0.05 / grid_resolution_));  // 约每5厘米一条线
  }

  // 绘制水平线
  for (int y = 0; y < grid_map_.rows; y += grid_stride)
  {
    cv::line(grid_map_, cv::Point(0, y), cv::Point(grid_map_.cols - 1, y), grid_color, line_width);
  }

  // 绘制垂直线
  for (int x = 0; x < grid_map_.cols; x += grid_stride)
  {
    cv::line(grid_map_, cv::Point(x, 0), cv::Point(x, grid_map_.rows - 1), grid_color, line_width);
  }
}

void RPPController::drawLookaheadPointOnGrid(const geometry_msgs::msg::Point& lookahead_point)
{
  if (grid_map_.empty())
    return;

  // 获取前瞻点的栅格坐标
  cv::Point grid_point = worldToGrid(lookahead_point.x, lookahead_point.y);

  // 检查点是否在栅格图范围内
  if (grid_point.x >= 0 && grid_point.x < grid_map_.cols && grid_point.y >= 0 && grid_point.y < grid_map_.rows)
  {
    // 使用紫色绘制前瞻点，用十字形状以便于区分
    cv::Scalar lookahead_color(255, 0, 255);  // 紫色
    int cross_size = 4;                       // 十字大小

    cv::line(grid_map_, cv::Point(grid_point.x - cross_size, grid_point.y),
             cv::Point(grid_point.x + cross_size, grid_point.y), lookahead_color, 1);

    cv::line(grid_map_, cv::Point(grid_point.x, grid_point.y - cross_size),
             cv::Point(grid_point.x, grid_point.y + cross_size), lookahead_color, 1);

    // 在十字周围添加一个小圆圈
    cv::circle(grid_map_, grid_point, cross_size, lookahead_color, 1);
  }
}

}  // namespace follow_controller
}  // namespace xline
