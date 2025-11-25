/**
 * @file rpp_follow_controller.cpp
 * @brief Regulated Pure Pursuit 路径跟随控制器实现
 *
 * 该控制器实现了改进的Pure Pursuit算法，支持：
 * - 曲线路径跟随
 * - 圆形路径跟随（带航向预对准）
 * - 后退模式
 * - 多种滤波和平滑处理
 */

#include "xline_follow_controller/rpp_follow_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace xline
{
namespace follow_controller
{

// ============================================================================
// 第一部分：构造函数、析构函数与初始化
// ============================================================================

/**
 * @brief RPPController 构造函数
 *
 * 完成以下工作：
 * - 调用基类构造函数，注册控制器名称
 * - 初始化各类状态标志、误差统计、路径与滤波参数等成员变量
 * - 调用 initialize() 完成运行期初始化
 * - 默认加载曲线路径参数配置文件（rpp_curve.yaml）
 */
RPPController::RPPController()
  : BaseFollowController("rpp_follow_controller")
  // 状态标志
  , initialized_(false)
  , goal_reached_(false)
  , waiting_(false)
  , is_circle_path(false)
  , back_follow_(false)
  // 航向预对准
  , need_yaw_prealign_(false)
  , yaw_prealign_done_(false)
  , target_yaw_(0.0)
  // 圆形路径参数
  , circle_entry_x_(0.0)
  , circle_entry_y_(0.0)
  , last_yaw_initialized_(false)
  , last_yaw_(0.0)
  , accumulated_angle_(0.0)
  , angle_debug_counter_(0)
  // 误差统计
  , max_error_(0.0)
  , avg_error_(0.0)
  , error_sum_(0.0)
  , error_count_(0)
  , current_lateral_error_(0.0)
  , current_curvature_(0.0)
  // 路径信息
  , path_length_(0.0)
  , traversed_distance_(0.0)
  , remaining_distance_(0.0)
  , last_closest_idx(0)
  // 栅格图参数
  , enable_grid_map_(false)
  , grid_resolution_(0.01)
  , grid_width_(10.0)
  , grid_height_(10.0)
  , grid_map_path_("/home/xline/zyq_ws")
  // 角速度滤波
  , previous_angular_vel_(0.0)
  , predicted_angular_vel_(0.0)
  , lowpass_angular_vel_filter_gain_(0.7)
  , angular_vel_history_size_(5)
  , angle_to_path_prev_(0.0)
  , lookahead_dist_prev_(0.0)
  // 二阶平滑器
  , second_order_filter_(2.0, 0.7)
{
  RCLCPP_INFO(get_logger(), "RPPController实例已创建，正在初始化...");
  initialize();
  updateParameters("/config/rpp_curve.yaml");
}

/**
 * @brief RPPController 析构函数
 *
 * 目前仅输出日志，释放时不做额外资源管理，
 * 主要用于调试查看控制器生命周期。
 */
RPPController::~RPPController()
{
  RCLCPP_INFO(get_logger(), "RPPController实例已销毁");
}

/**
 * @brief 控制器运行期初始化
 *
 * - 根据控制频率设置控制周期 d_t_
 * - 如启用栅格图可视化，则创建保存目录
 * - 设置 initialized_ 标志，避免重复初始化
 */
void RPPController::initialize()
{
  if (initialized_)
  {
    return;
  }

  // 计算时间步长 - 18Hz更新频率
  d_t_ = 1.0 / 18.0;

  // 创建栅格图保存目录
  if (enable_grid_map_)
  {
    std::filesystem::create_directories(grid_map_path_);
    RCLCPP_INFO(get_logger(), "栅格图将保存到: %s", grid_map_path_.c_str());
  }

  initialized_ = true;
  RCLCPP_INFO(get_logger(), "控制器初始化完成，控制周期: %.4fs", d_t_);
}

// ============================================================================
// 第二部分：参数管理
// ============================================================================

/**
 * @brief 从 YAML 配置文件中加载控制参数
 *
 * 通过 ament_index_cpp 获取包共享目录，拼接外部传入的配置文件相对路径，
 * 使用自定义的 YamlParser 读取纯跟踪算法相关的所有参数，包括：
 * - 目标判定阈值（距离、角度）
 * - 前瞻距离与时间
 * - 线速度 / 角速度上下限及加加速度约束
 * - 曲率约束、接近目标减速参数
 * - 角速度平滑与二阶滤波参数
 * - 位置与角速度滤波器参数
 * - 圆形路径偏差修正参数
 * - 栅格图开关与角速度增益等
 *
 * @param file_path 配置文件相对路径（以包共享目录为基准）
 */
void RPPController::updateParameters(std::string file_path)
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("xline_follow_controller");
  std::string config_file_path = package_share_directory + file_path;
  xline::YamlParser::YamlParser parser(config_file_path);

  // 位置和方向容忍度
  goal_dist_tol_ = parser.getParameter<double>("goal_dist_tol");
  rotate_tol_ = parser.getParameter<double>("rotate_tol");

  // 前瞻参数
  lookahead_time_ = parser.getParameter<double>("lookahead_time");
  min_lookahead_dist_ = parser.getParameter<double>("min_lookahead_dist");
  max_lookahead_dist_ = parser.getParameter<double>("max_lookahead_dist");

  // 线速度参数
  max_v_ = parser.getParameter<double>("max_v");
  min_v_ = parser.getParameter<double>("min_v");
  max_v_inc_ = parser.getParameter<double>("max_v_inc");
  linear_speed_ = parser.getParameter<double>("linear_speed");

  // 角速度参数
  max_w_ = parser.getParameter<double>("max_w");
  min_w_ = parser.getParameter<double>("min_w");
  max_w_inc_ = parser.getParameter<double>("max_w_inc");

  // 路径跟踪参数
  regulated_min_radius_ = parser.getParameter<double>("regulated_min_radius");
  approach_dist_ = parser.getParameter<double>("approach_dist");
  approach_min_v_ = parser.getParameter<double>("approach_min_v");

  // 角速度平滑参数
  radius_offset_ = parser.getParameter<double>("radius_offset");
  lowpass_angular_vel_filter_gain_ = parser.getParameter<double>("lowpass_angular_vel_filter_gain");
  smoothing_type_ = parser.getParameter<std::string>("smoothing_type");
  angular_vel_history_size_ = parser.getParameter<int>("angular_vel_history_size");

  // 二阶平滑器参数
  angular_smoother_freq_ = parser.getParameter<double>("smoother_freq");
  angular_smoother_damping_ = parser.getParameter<double>("smoother_damping");
  second_order_filter_.setParameters(angular_smoother_freq_, angular_smoother_damping_);

  // 位置滤波参数
  pos_cutoff_freq = parser.getParameter<double>("pos_cutoff_freq");
  pos_sample_rate = parser.getParameter<double>("pos_sample_rate");
  pos_output_limit = parser.getParameter<double>("pos_output_limit");
  pos_rate_limit = parser.getParameter<double>("pos_rate_limit");
  pos_use_biquad_cascade_ = parser.getParameter<bool>("pos_use_biquad_cascade");
  pos_use_biquad_cascade_filter_ = parser.getParameter<bool>("pos_use_biquad_cascade_filter");

  // 角速度滤波参数
  angle_cutoff_freq = parser.getParameter<double>("angle_cutoff_freq");
  angle_sample_rate = parser.getParameter<double>("angle_sample_rate");
  angle_output_limit_rate = parser.getParameter<double>("angle_output_limit_rate");
  angle_rate_limit = parser.getParameter<double>("angle_rate_limit");
  angle_use_biquad_cascade_ = parser.getParameter<bool>("angle_use_biquad_cascade");
  angle_use_biquad_cascade_filter_ = parser.getParameter<bool>("angle_use_biquad_cascade_filter");
  angle_use_offset_limit_ = parser.getParameter<bool>("angle_use_offset_limit");
  angle_output_offset_ = parser.getParameter<double>("angle_output_offset");

  // 圆形路径偏差参数
  start_deviation_factor_ = parser.getParameter<double>("start_deviation_factor");
  end_deviation_factor_ = parser.getParameter<double>("end_deviation_factor");
  deviation_rate_ = parser.getParameter<double>("deviation_rate");

  // 其他参数
  enable_grid_map_ = parser.getParameter<bool>("enable_grid_map");
  low_speed_mode_ = parser.getParameter<bool>("low_speed_mode");

  // 记录关键参数
  RCLCPP_INFO(get_logger(),
              "参数已更新: 前瞻距离[%.2f-%.2f]m, 速度[%.2f-%.2f]m/s, 角速度[%.2f-%.2f]rad/s",
              min_lookahead_dist_, max_lookahead_dist_, min_v_, max_v_, min_w_, max_w_);
}

// ============================================================================
// 第三部分：路径设置
// ============================================================================

/**
 * @brief 设置圆形路径的起止角度范围
 *
 * 用于限定机器人在圆周上行驶的总角度，内部根据起止角度
 * 额外增加 0.6π 的冗余，以便在圆周起止处留有缓冲区。
 *
 * @param start_angle 圆周起始角度（弧度）
 * @param end_angle 圆周结束角度（弧度）
 */
void RPPController::setAngleRange(double start_angle, double end_angle)
{
  circle_start_angle = start_angle;
  circle_end_angle = end_angle;
  circle_total_angle = std::abs(circle_end_angle - circle_start_angle) + 0.8 * M_PI;
}

/**
 * @brief 设置圆形路径跟随计划
 *
 * 使用圆心和半径描述圆形路径，并结合当前机器人位姿生成一条
 * 「直线切入 + 圆周运动」的目标路径，同时根据半径自动调整
 * 跟踪参数和基准角速度。
 *
 * @param circle_center_x 圆心 x 坐标（世界坐标系）
 * @param circle_center_y 圆心 y 坐标（世界坐标系）
 * @param circle_radius 圆半径
 * @param robot_pose 当前机器人位姿
 * @return 设置成功返回 true
 */
bool RPPController::setPlanForCircle(double circle_center_x, double circle_center_y, double circle_radius,
                                     const geometry_msgs::msg::PoseStamped& robot_pose)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化，请先调用initialize()");
    return false;
  }

  if (circle_radius <= 0.0)
  {
    RCLCPP_ERROR(get_logger(), "圆半径必须为正值");
    return false;
  }

  updateParameters("/config/rpp_circle.yaml");
  back_follow_ = false;  // 圆形路径不使用后退模式

  // 根据半径调整速度参数（非低速模式时）
  if (!low_speed_mode_)
  {
    adjustSpeedForRadius(circle_radius);
  }

  // 应用半径偏移
  circle_radius += radius_offset_;
  is_circle_path = true;

  // 重置角度累计相关变量
  resetCirclePathState();

  // 存储圆形路径参数
  circle_center_x_ = circle_center_x;
  circle_center_y_ = circle_center_y;
  circle_radius_ = circle_radius;
  baseline_angular_velocity_for_circle_ = min_v_ / circle_radius;

  try
  {
    // 生成圆形路径
    nav_msgs::msg::Path circle_path = generateCirclePath(circle_center_x, circle_center_y, circle_radius, robot_pose);

    RCLCPP_INFO(get_logger(), "圆形路径已生成 - 圆心: (%.3f, %.3f), 半径: %.3f m, 点数: %zu",
                circle_center_x, circle_center_y, circle_radius, circle_path.poses.size());

    return setPlan(circle_path);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "设置圆形路径时发生异常: %s", e.what());
    return false;
  }
}

/**
 * @brief 设置通用路径跟随计划
 *
 * 支持任意 nav_msgs::Path 类型路径：
 * - 首先重置内部状态
 * - 对路径进行插值和平滑（点数>2时）
 * - 清理掉非法点并统计路径长度和目标信息
 * - 如启用栅格图，可同时初始化可视化底图
 *
 * @param orig_global_plan 外部传入的原始全局路径（世界坐标系）
 * @return 设置成功返回 true
 */
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

  // 重置控制器状态
  resetControllerState();

  // 处理和优化路径
  if (orig_global_plan.poses.size() > 2)
  {
    global_plan_ = smoothAndSubdividePath(orig_global_plan);
    RCLCPP_INFO(get_logger(), "路径优化完成: 原始点数=%zu, 优化后点数=%zu",
                orig_global_plan.poses.size(), global_plan_.poses.size());
  }
  else
  {
    global_plan_ = orig_global_plan;
  }

  // 验证并清理路径中的无效点
  if (!validateAndCleanPath())
  {
    return false;
  }

  // 计算路径信息
  calculatePathInfo();

  // 初始化栅格图
  if (enable_grid_map_)
  {
    initializeGridMap(global_plan_);
  }

  return true;
}

/**
 * @brief 设置是否启用后退跟随模式
 *
 * 后退模式下会将机器人位姿进行等效变换，使控制逻辑仍然
 * 按“前进”实现，但输出线速度取反。
 *
 * @param back true 表示启用后退跟随
 */
void RPPController::setBackFollow(bool back)
{
  back_follow_ = back;
  need_yaw_prealign_ = true;
  yaw_prealign_done_ = false;
  RCLCPP_INFO(get_logger(), "设置后退模式: %s", back ? "启用" : "禁用");
}

/**
 * @brief 查询是否已经到达目标
 *
 * 该接口主要供上层规划器调用，用于判断当前控制器是否完成任务。
 * 内部会在首次到达目标时输出一次统计信息（最大/平均误差）。
 *
 * @return 已到达目标返回 true
 */
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
      RCLCPP_INFO(get_logger(), "目标已到达 - 最大误差: %.4fm, 平均误差: %.4fm", max_error_, avg_error_);
      reported = true;
    }
    return true;
  }

  return false;
}

// ============================================================================
// 第四部分：核心速度计算
// ============================================================================

/**
 * @brief 计算当前时刻的速度控制指令
 *
 * 这是控制器的核心入口函数，完成流程如下：
 * 1. 对输入位姿进行滤波与合法性检查
 * 2. 如启用航向预对准（圆形路径或后退模式），优先进行原地转向
 * 3. 在后退模式下对位姿进行等效变换
 * 4. 检查是否已到达目标或圆形路径结束
 * 5. 根据当前位姿裁剪路径，计算前瞻点与角度误差、曲率和横向误差
 * 6. 根据曲率等信息计算期望线速度和角速度
 * 7. 生成最终速度指令并更新栅格图与性能统计
 *
 * @param robot_pose 当前机器人位姿
 * @param current_velocity 当前机器人速度
 * @param cmd_vel 输出的速度指令
 * @return 计算成功返回 true
 */
bool RPPController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& robot_pose,
                                            const geometry_msgs::msg::Twist& current_velocity,
                                            geometry_msgs::msg::TwistStamped& cmd_vel)
{
  if (!initialized_ || global_plan_.poses.empty())
  {
    RCLCPP_ERROR(get_logger(), "控制器未初始化或路径为空");
    return false;
  }

  // 初始化输出命令
  initializeCommandVel(cmd_vel);

  // 检查是否已到达目标
  if (goal_reached_)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "目标已达到，保持停止状态");
    return true;
  }

  // 处理等待状态
  static int waiting_count = 0;
  if (waiting_)
  {
    if (++waiting_count > 10)
    {
      waiting_count = 0;
      waiting_ = false;
    }
    return true;
  }

  // 位置滤波处理
  geometry_msgs::msg::PoseStamped current_pose = filterRobotPose(robot_pose);
  current_velocity_ = current_velocity;

  static bool filter_reset = false;
  try
  {
    auto start_time = this->now();

    // 位置有效性检查
    if (!isValidPose(current_pose))
    {
      RCLCPP_ERROR(get_logger(), "接收到无效的机器人位置: (%.3f, %.3f)",
                   current_pose.pose.position.x, current_pose.pose.position.y);
      return false;
    }

    // 航向预对准处理（圆形路径或后退模式）
    if ((is_circle_path || back_follow_) && need_yaw_prealign_ && !yaw_prealign_done_)
    {
      if (performYawPrealignment(current_pose, target_yaw_, cmd_vel))
      {
        yaw_prealign_done_ = true;
        need_yaw_prealign_ = false;
        waiting_ = true;
        RCLCPP_INFO(get_logger(), "航向预对准完成，开始路径跟随");
      }
      return true;
    }

    // 后退模式下调整航向角
    if (back_follow_)
    {
      current_pose = adjustPoseForBackward(current_pose);
    }

    // 检查是否到达目标
    if (checkGoalReached(current_pose, cmd_vel))
    {
      return true;
    }

    // 裁剪全局路径
    std::vector<geometry_msgs::msg::PoseStamped> pruned_plan;
    pruneGlobalPlan(current_pose, global_plan_, pruned_plan);

    if (pruned_plan.empty())
    {
      RCLCPP_ERROR(get_logger(), "裁剪后的路径为空");
      return false;
    }

    // 获取前瞻点
    double lookahead_distance = getLookAheadDistance(current_velocity.linear.x);
    geometry_msgs::msg::PoseStamped lookahead_pose = getLookAheadPoint(lookahead_distance, pruned_plan, true);

    if (!isValidPose(lookahead_pose))
    {
      lookahead_pose = getFallbackLookaheadPoint(pruned_plan);
      if (!isValidPose(lookahead_pose))
      {
        return false;
      }
    }

    // 计算控制量
    double angle_to_lookahead = computeAngleToLookahead(lookahead_pose, current_pose);
    current_curvature_ = computeCurvature(angle_to_lookahead, lookahead_distance);
    current_lateral_error_ = computeLateralError(angle_to_lookahead, lookahead_distance);

    // 更新误差统计
    updateErrorStatistics();

    // 计算期望速度
    double desired_velocity = applyCurvatureConstraint(linear_speed_, current_curvature_);
    double desired_angular_velocity = computeDesiredAngularVelocity(
        desired_velocity, current_curvature_, current_velocity.angular.z,
        lookahead_distance, angle_to_lookahead, d_t_, filter_reset);

    // 生成速度命令
    generateVelocityCommand(cmd_vel, current_velocity, desired_velocity, desired_angular_velocity);

    // 更新栅格图
    updateGridMapIfNeeded(current_pose, lookahead_pose);

    // 性能监控
    checkComputationTime(start_time);

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "计算速度命令时发生异常: %s", e.what());
    return false;
  }
}

// ============================================================================
// 第五部分：路径处理辅助函数
// ============================================================================

/**
 * @brief 根据当前位姿裁剪全局路径
 *
 * 在全局路径中找到距离当前位姿最近的路径点，并将该点之后的
 * 所有点拷贝到裁剪后的路径中，用于降低无关历史路径对控制的影响。
 *
 * @param current_pose 当前机器人位姿
 * @param global_plan 原始全局路径
 * @param pruned_plan 裁剪后的路径输出
 */
void RPPController::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& current_pose,
                                    const nav_msgs::msg::Path& global_plan,
                                    std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan)
{
  pruned_plan.clear();

  if (global_plan.poses.empty())
  {
    return;
  }

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

  // 添加最近点及后续所有点
  pruned_plan.insert(pruned_plan.end(), global_plan.poses.begin() + closest_idx, global_plan.poses.end());
}

/**
 * @brief 沿裁剪后路径查找指定前瞻距离处的前瞻点
 *
 * 通过在路径上累积弧长，当累计距离首次超过前瞻距离时，
 * 在当前段内进行线性插值，计算精确的几何前瞻点，并根据
 * 路径切线方向设置前瞻点的朝向。
 *
 * @param lookahead_dist 目标前瞻距离
 * @param transformed_plan 已在世界坐标系中的路径
 * @param interpolate_after_goal 是否允许在目标之后继续插值（此处主要保留接口形态）
 * @return 计算得到的前瞻点位姿
 */
geometry_msgs::msg::PoseStamped RPPController::getLookAheadPoint(
    const double& lookahead_dist, const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
    bool interpolate_after_goal)
{
  if (transformed_plan.empty())
  {
    RCLCPP_ERROR(get_logger(), "接收到空路径，无法计算前瞻点");
    return geometry_msgs::msg::PoseStamped();
  }

  if (transformed_plan.size() == 1)
  {
    return transformed_plan[0];
  }

  // 沿路径累积距离查找前瞻点
  double accumulated_dist = 0.0;

  for (size_t i = 0; i < transformed_plan.size() - 1; ++i)
  {
    double segment_length = std::hypot(
        transformed_plan[i + 1].pose.position.x - transformed_plan[i].pose.position.x,
        transformed_plan[i + 1].pose.position.y - transformed_plan[i].pose.position.y);

    if (accumulated_dist + segment_length >= lookahead_dist)
    {
      double remaining_dist = lookahead_dist - accumulated_dist;
      double ratio = remaining_dist / segment_length;

      geometry_msgs::msg::PoseStamped lookahead_pose;
      lookahead_pose.header = transformed_plan[i].header;
      lookahead_pose.pose.position.x = transformed_plan[i].pose.position.x +
          ratio * (transformed_plan[i + 1].pose.position.x - transformed_plan[i].pose.position.x);
      lookahead_pose.pose.position.y = transformed_plan[i].pose.position.y +
          ratio * (transformed_plan[i + 1].pose.position.y - transformed_plan[i].pose.position.y);

      // 计算朝向
      double yaw = atan2(transformed_plan[i + 1].pose.position.y - transformed_plan[i].pose.position.y,
                         transformed_plan[i + 1].pose.position.x - transformed_plan[i].pose.position.x);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      lookahead_pose.pose.orientation = tf2::toMsg(q);

      return lookahead_pose;
    }

    accumulated_dist += segment_length;
  }

  // 前瞻距离超出路径长度，返回最后一个点
  return transformed_plan.back();
}

/**
 * @brief 根据当前速度与误差自适应计算前瞻距离
 *
 * 前瞻距离由多因素共同决定：
 * - 基础项：线速度乘以前瞻时间
 * - 误差修正：横向误差过大时适当缩短前瞻距离以增强纠偏能力
 * - 曲率修正：曲率过大（急转弯）时减小前瞻距离以防过冲
 * - 目标修正：接近路径终点时逐步减小前瞻距离以平滑停车
 *
 * @param speed 当前线速度
 * @return 建议前瞻距离
 */
double RPPController::getLookAheadDistance(double speed)
{
  double base_lookahead_dist = std::fabs(speed) * lookahead_time_;

  // 根据横向误差动态调整
  double error_factor = 1.0;
  if (std::fabs(current_lateral_error_) > 0.1)
  {
    error_factor = 1.0 - std::min(0.3, std::fabs(current_lateral_error_) * 0.5);
  }

  double lookahead_dist = base_lookahead_dist * error_factor;
  lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);

  // 高曲率时减小前瞻距离
  if (std::fabs(current_curvature_) > 5.0)
  {
    lookahead_dist = std::max(min_lookahead_dist_, lookahead_dist * 0.8);
  }

  // 接近目标时调整
  if (remaining_distance_ < lookahead_dist * 2.0)
  {
    lookahead_dist = std::max(min_lookahead_dist_, remaining_distance_ / 2.0);
  }

  return lookahead_dist;
}

// ============================================================================
// 第六部分：速度计算辅助函数
// ============================================================================

/**
 * @brief 计算机器人到前瞻点的角度差（世界坐标系）
 *
 * 通过前瞻点与当前位姿的连线方向与当前航向角之差，
 * 得到需要转向的角度误差，并将结果规范到 [-π, π]。
 *
 * @param lookahead_pt 前瞻点（PointStamped）
 * @param robot_pose_global 当前机器人位姿（PoseStamped）
 * @return 角度差（弧度）
 */
double RPPController::dphi(geometry_msgs::msg::PointStamped lookahead_pt, geometry_msgs::msg::PoseStamped robot_pose)
{
  double global_angle = atan2(lookahead_pt.point.y - robot_pose.pose.position.y,
                              lookahead_pt.point.x - robot_pose.pose.position.x);
  double robot_angle = tf2::getYaw(robot_pose.pose.orientation);
  return regularizeAngle(global_angle - robot_angle);
}

/**
 * @brief 将角度值规范到 [-π, π] 区间
 *
 * 可避免由于角度累积导致的数值跳变问题，保证角度运算连续。
 *
 * @param angle 原始角度
 * @return 规范化后的角度
 */
double RPPController::regularizeAngle(double angle)
{
  return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

/**
 * @brief 根据路径曲率约束线速度
 *
 * 曲率越大（半径越小）时控制器会主动减小线速度，
 * 以保证曲线行驶时的稳定性和安全性；在极高曲率场景下
 * 还会进一步限制最大速度。
 *
 * @param raw_linear_vel 原始线速度（未约束）
 * @param curvature 当前路径曲率
 * @return 约束后的线速度
 */
double RPPController::applyCurvatureConstraint(const double raw_linear_vel, const double curvature)
{
  if (std::fabs(curvature) < 1e-10)
  {
    return raw_linear_vel;
  }

  double radius = 1.0 / std::fabs(curvature);
  double constrained_vel = raw_linear_vel;

  if (radius < regulated_min_radius_)
  {
    double ratio = (radius / regulated_min_radius_);
    ratio = ratio * ratio;  // 二次关系
    constrained_vel = std::max(min_v_, raw_linear_vel * ratio);
  }

  // 极高曲率时进一步限制
  if (std::fabs(curvature) > 5.0)
  {
    constrained_vel = std::min(constrained_vel, min_v_ * 1.2);
  }

  return constrained_vel;
}

/**
 * @brief 根据距离终点的剩余距离对线速度进行减速约束
 *
 * 当剩余路径长度小于接近距离阈值时，控制器会根据与终点的距离
 * 逐渐降低线速度，防止临近目标时出现冲过或振荡。
 *
 * @param raw_linear_vel 原始线速度
 * @param robot_pose 当前机器人位姿
 * @param pruned_plan 裁剪后的路径
 * @return 约束后的线速度
 */
double RPPController::applyApproachConstraint(const double raw_linear_vel,
                                              geometry_msgs::msg::PoseStamped robot_pose,
                                              const std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan)
{
  if (pruned_plan.empty())
  {
    return raw_linear_vel;
  }

  // 计算剩余路径长度
  double remaining_length = 0.0;
  for (size_t i = 0; i < pruned_plan.size() - 1; ++i)
  {
    remaining_length += std::hypot(pruned_plan[i + 1].pose.position.x - pruned_plan[i].pose.position.x,
                                   pruned_plan[i + 1].pose.position.y - pruned_plan[i].pose.position.y);
  }

  if (remaining_length < approach_dist_)
  {
    double distance_to_end = std::hypot(pruned_plan.back().pose.position.x - robot_pose.pose.position.x,
                                        pruned_plan.back().pose.position.y - robot_pose.pose.position.y);
    double ratio = distance_to_end / approach_dist_;
    return std::max(approach_min_v_, raw_linear_vel * ratio);
  }

  return raw_linear_vel;
}

/**
 * @brief 对线速度进行加速度与上下限约束
 *
 * 限制每个控制周期内线速度的变化量不超过 max_v_inc_，
 * 同时保证最终线速度位于 [min_v_, max_v_] 区间，以避免
 * 突然加减速带来的不适与滑移。
 *
 * @param current_velocity 当前线速度
 * @param desired_velocity 期望线速度
 * @return 正则化后的线速度
 */
double RPPController::linearRegularization(double current_velocity, double desired_velocity)
{
  double velocity_increment = desired_velocity - current_velocity;

  if (std::fabs(velocity_increment) > max_v_inc_)
  {
    velocity_increment = std::copysign(max_v_inc_, velocity_increment);
  }

  double command_velocity = current_velocity + velocity_increment;

  if (std::fabs(command_velocity) > max_v_)
  {
    command_velocity = std::copysign(max_v_, command_velocity);
  }
  else if (std::fabs(command_velocity) < min_v_ && desired_velocity != 0.0)
  {
    command_velocity = std::copysign(min_v_, command_velocity);
  }

  return command_velocity;
}

/**
 * @brief 对角速度进行限幅与加加速度约束
 *
 * 根据角速度误差动态调整允许的最大角加速度，并施加
 * 上下限约束，避免在转弯时出现过大的角速度突变。
 *
 * @param current_angular_vel 当前角速度
 * @param desired_angular_vel 期望角速度
 * @return 正则化后的角速度
 */
double RPPController::angularRegularization(double current_angular_vel, double desired_angular_vel)
{
  double max_allowed_w = max_w_;

  if (std::fabs(desired_angular_vel) > max_w_ * 0.8)
  {
    max_allowed_w = max_w_ * 0.8;
  }

  if (std::fabs(desired_angular_vel) > max_allowed_w)
  {
    desired_angular_vel = std::copysign(max_allowed_w, desired_angular_vel);
  }

  double angular_increment = desired_angular_vel - current_angular_vel;

  // 动态调整角加速度限制
  double effective_max_w_inc = max_w_inc_;
  if (std::fabs(angular_increment) > max_w_inc_ * 2.0)
  {
    effective_max_w_inc = max_w_inc_ * 1.5;
  }
  else if (std::fabs(angular_increment) < max_w_inc_ * 0.5)
  {
    effective_max_w_inc = max_w_inc_ * 0.8;
  }

  if (std::fabs(angular_increment) > effective_max_w_inc)
  {
    angular_increment = std::copysign(effective_max_w_inc, angular_increment);
  }

  double command_angular_vel = current_angular_vel + angular_increment;

  if (std::fabs(command_angular_vel) > max_allowed_w)
  {
    command_angular_vel = std::copysign(max_allowed_w, command_angular_vel);
  }
  else if (std::fabs(command_angular_vel) < min_w_ && desired_angular_vel != 0.0)
  {
    command_angular_vel = std::copysign(min_w_, command_angular_vel);
  }

  return command_angular_vel;
}

/**
 * @brief 判断是否需要原地旋转以对齐路径方向
 *
 * 当当前朝向与路径方向之间的角度误差大于给定阈值时，
 * 需要优先执行旋转动作再进行前向跟踪。
 *
 * @param angle_to_path 当前路径方向误差
 * @param tolerance 允许的误差容忍度（若 <=0 则使用默认 rotate_tol_）
 * @return 需要旋转返回 true
 */
bool RPPController::shouldRotateToPath(double angle_to_path, double tolerance)
{
  double angle_threshold = tolerance > 0.0 ? tolerance : rotate_tol_;
  return std::fabs(angle_to_path) > angle_threshold;
}

/**
 * @brief 判断是否需要对齐到目标朝向
 *
 * 当机器人已经接近目标位置（距离小于 goal_dist_tol_）时，
 * 进入原地旋转阶段，对齐目标姿态。
 *
 * @param current_pose 当前机器人位姿
 * @param goal_pose 目标位姿
 * @return 需要旋转对齐目标返回 true
 */
bool RPPController::shouldRotateToGoal(const geometry_msgs::msg::PoseStamped& current_pose,
                                       const geometry_msgs::msg::PoseStamped& goal_pose)
{
  double distance_to_goal = std::hypot(current_pose.pose.position.x - goal_pose.pose.position.x,
                                       current_pose.pose.position.y - goal_pose.pose.position.y);
  return distance_to_goal < goal_dist_tol_;
}

// ============================================================================
// 第七部分：航向预对准与旋转控制
// ============================================================================

/**
 * @brief 在启用圆形路径或后退模式时执行航向预对准
 *
 * 通过原地旋转的方式，使机器人当前航向与目标航向对齐，
 * 以减少进入路径时的初始偏差，避免刚起步阶段的强烈转向。
 *
 * @param current_pose 当前机器人位姿
 * @param target_yaw 预期航向角
 * @param cmd_vel 输出的速度指令（仅使用角速度）
 * @return 当航向误差小于阈值时返回 true
 */
bool RPPController::performYawPrealignment(const geometry_msgs::msg::PoseStamped& current_pose,
                                           double target_yaw,
                                           geometry_msgs::msg::TwistStamped& cmd_vel)
{
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double yaw_error = angles::shortest_angular_distance(current_yaw, target_yaw);
  double yaw_tolerance = 0.05;  // 约3度

  if (std::abs(yaw_error) < yaw_tolerance)
  {
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
                       "航向预对准中... 目标: %.3f, 当前: %.3f, 误差: %.3f, 角速度: %.3f",
                       target_yaw, current_yaw, yaw_error, angular_vel);

  return false;
}

/**
 * @brief 根据角度误差计算预对准阶段的角速度
 *
 * 使用类 Sigmoid + 余弦平滑的非线性函数，使得：
 * - 角度误差较大时角速度接近常值，加快收敛
 * - 接近目标时角速度逐渐减小，避免过冲与振荡
 *
 * @param angle_diff 当前角度误差
 * @return 建议角速度（符号与 angle_diff 一致）
 */
double RPPController::calculateRotationVelocity(const double& angle_diff)
{
  double factor = 1.0 / (1.0 + std::exp(-1.0 * std::abs(angle_diff)));

  // 接近目标时使用余弦曲线平滑减速
  if (std::abs(angle_diff) < 0.5)
  {
    double cosine_factor = 0.5 * (1.0 - std::cos(M_PI * std::abs(angle_diff) / 0.5));
    factor *= cosine_factor;
  }

  double rot_vel = std::max(1.0 * factor, 0.2);
  return (angle_diff > 0.0) ? rot_vel : -rot_vel;
}

// ============================================================================
// 第八部分：角速度平滑
// ============================================================================

/**
 * @brief 对角速度进行多级平滑处理
 *
 * 根据配置选择不同的平滑方式（低通 / 滑动平均），再通过
 * 二阶滤波器与可选的四阶低通滤波进一步平滑角速度，使其
 * 在保证响应性的前提下尽量平稳。
 *
 * @param current_angular_vel 当前角速度
 * @param desired_angular_vel 期望角速度
 * @param lookahead_dist 当前前瞻距离
 * @param angle_to_path 当前路径角度误差
 * @param dt 控制周期
 * @param is_reset 是否重置滤波器历史数据
 * @return 平滑后的角速度
 */
double RPPController::smoothAngularVelocity(double current_angular_vel, double desired_angular_vel,
                                            double lookahead_dist, double angle_to_path, double dt, bool is_reset)
{
  if (is_reset)
  {
    angular_vel_history_.clear();
    second_order_filter_.reset();
    previous_angular_vel_ = 0.0;
  }

  // 更新历史记录
  angular_vel_history_.push_back(desired_angular_vel);
  if (angular_vel_history_.size() > static_cast<size_t>(angular_vel_history_size_))
  {
    angular_vel_history_.pop_front();
  }

  double smoothed_angular_vel = desired_angular_vel;

  // 应用选定的平滑方法
  if (smoothing_type_ == "lowpass")
  {
    smoothed_angular_vel = lowpass_angular_vel_filter_gain_ * desired_angular_vel +
                           (1 - lowpass_angular_vel_filter_gain_) * previous_angular_vel_;
  }
  else if (smoothing_type_ == "movingAverage")
  {
    smoothed_angular_vel = 0.0;
    for (double vel : angular_vel_history_)
    {
      smoothed_angular_vel += vel;
    }
    smoothed_angular_vel /= angular_vel_history_.size();
  }

  // 二阶平滑器
  smoothed_angular_vel = second_order_filter_.filter(smoothed_angular_vel, dt);

  // 四阶低通滤波
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

// ============================================================================
// 第九部分：栅格图可视化
// ============================================================================

/**
 * @brief 使用当前路径初始化栅格地图
 *
 * 计算路径的边界框和合适的分辨率，创建 OpenCV 图像作为地图，
 * 绘制网格线与路径，并立即保存一次，用于离线观察规划效果。
 *
 * @param path 当前全局路径
 */
void RPPController::initializeGridMap(const nav_msgs::msg::Path& path)
{
  if (path.poses.empty())
  {
    RCLCPP_WARN(get_logger(), "无法初始化栅格图：路径为空");
    return;
  }

  // 计算路径边界框
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

  // 添加边距
  double margin = 1.0;
  grid_width_ = (max_x - min_x) + 2 * margin;
  grid_height_ = (max_y - min_y) + 2 * margin;
  grid_origin_x_ = min_x - margin;
  grid_origin_y_ = min_y - margin;

  // 创建栅格图
  int width_pixels = static_cast<int>(grid_width_ / grid_resolution_);
  int height_pixels = static_cast<int>(grid_height_ / grid_resolution_);
  grid_map_ = cv::Mat(height_pixels, width_pixels, CV_8UC3, cv::Scalar(255, 255, 255));

  drawGridLines();
  drawPathOnGrid(path, cv::Scalar(0, 0, 255), 1);

  last_grid_update_time_ = this->now();
  saveGridMap();

  RCLCPP_INFO(get_logger(), "初始化栅格图 大小: %.2f x %.2f m, 尺寸: %d x %d 像素",
              grid_width_, grid_height_, width_pixels, height_pixels);
}

/**
 * @brief 将世界坐标转换为栅格图像素坐标
 *
 * 根据栅格原点与分辨率，将 (x, y) 映射到图像中的 (u, v)。
 *
 * @param x 世界坐标系下的 x
 * @param y 世界坐标系下的 y
 * @return 对应的像素坐标
 */
cv::Point RPPController::worldToGrid(double x, double y)
{
  int grid_x = static_cast<int>((x - grid_origin_x_) / grid_resolution_);
  int grid_y = grid_map_.rows - static_cast<int>((y - grid_origin_y_) / grid_resolution_) - 1;
  return cv::Point(grid_x, grid_y);
}

/**
 * @brief 在栅格图上绘制路径
 *
 * 将路径中的相邻点连线，同时高亮起点与终点，用于可视化
 * 全局规划结果。
 *
 * @param path 要绘制的路径
 * @param color 线条颜色
 * @param thickness 线宽
 */
void RPPController::drawPathOnGrid(const nav_msgs::msg::Path& path, const cv::Scalar& color, int thickness)
{
  if (path.poses.empty() || grid_map_.empty())
  {
    return;
  }

  for (size_t i = 0; i < path.poses.size() - 1; ++i)
  {
    cv::Point pt1 = worldToGrid(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
    cv::Point pt2 = worldToGrid(path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y);

    if (isPointInGrid(pt1) && isPointInGrid(pt2))
    {
      cv::line(grid_map_, pt1, pt2, color, thickness);
    }
  }

  // 标记起点（绿色）和终点（蓝色）
  if (!path.poses.empty())
  {
    cv::Point start = worldToGrid(path.poses.front().pose.position.x, path.poses.front().pose.position.y);
    cv::Point end = worldToGrid(path.poses.back().pose.position.x, path.poses.back().pose.position.y);

    if (isPointInGrid(start))
    {
      cv::circle(grid_map_, start, 3, cv::Scalar(0, 255, 0), -1);
    }
    if (isPointInGrid(end))
    {
      cv::circle(grid_map_, end, 3, cv::Scalar(255, 0, 0), -1);
    }
  }
}

/**
 * @brief 在栅格图上绘制当前机器人位置
 *
 * 以一个小圆点标记机器人投影位置，用于观察跟踪效果。
 *
 * @param pose 机器人当前位姿
 */
void RPPController::drawRobotOnGrid(const geometry_msgs::msg::PoseStamped& pose)
{
  if (grid_map_.empty())
  {
    return;
  }

  cv::Point robot_pos = worldToGrid(pose.pose.position.x, pose.pose.position.y);

  if (isPointInGrid(robot_pos))
  {
    cv::circle(grid_map_, robot_pos, 1, cv::Scalar(0, 0, 0), -1);
  }
}

/**
 * @brief 在栅格图上绘制当前前瞻点
 *
 * 使用十字形标记当前纯跟踪算法使用的前瞻点位置，
 * 便于调试前瞻距离和路径状态。
 *
 * @param lookahead_point 前瞻点位置
 */
void RPPController::drawLookaheadPointOnGrid(const geometry_msgs::msg::Point& lookahead_point)
{
  if (grid_map_.empty())
  {
    return;
  }

  cv::Point grid_point = worldToGrid(lookahead_point.x, lookahead_point.y);

  if (isPointInGrid(grid_point))
  {
    cv::Scalar lookahead_color(255, 0, 255);  // 紫色
    int cross_size = 4;

    cv::line(grid_map_, cv::Point(grid_point.x - cross_size, grid_point.y),
             cv::Point(grid_point.x + cross_size, grid_point.y), lookahead_color, 1);
    cv::line(grid_map_, cv::Point(grid_point.x, grid_point.y - cross_size),
             cv::Point(grid_point.x, grid_point.y + cross_size), lookahead_color, 1);
    cv::circle(grid_map_, grid_point, cross_size, lookahead_color, 1);
  }
}

void RPPController::drawGridLines()
{
  if (grid_map_.empty())
  {
    return;
  }

  cv::Scalar grid_color(220, 220, 220);
  int grid_stride = std::max(1, static_cast<int>(0.05 / grid_resolution_));

  for (int y = 0; y < grid_map_.rows; y += grid_stride)
  {
    cv::line(grid_map_, cv::Point(0, y), cv::Point(grid_map_.cols - 1, y), grid_color, 1);
  }

  for (int x = 0; x < grid_map_.cols; x += grid_stride)
  {
    cv::line(grid_map_, cv::Point(x, 0), cv::Point(x, grid_map_.rows - 1), grid_color, 1);
  }
}

void RPPController::saveGridMap()
{
  if (grid_map_.empty())
  {
    return;
  }

  try
  {
    std::string latest_filename = grid_map_path_ + "/grid_map_latest.png";
    cv::imwrite(latest_filename, grid_map_);
  }
  catch (const cv::Exception& e)
  {
    RCLCPP_ERROR(get_logger(), "保存栅格图失败: %s", e.what());
  }
}

// ============================================================================
// 第十部分：私有辅助函数
// ============================================================================

/**
 * @brief 重置控制器内部状态
 *
 * 在接收新的路径计划或重新规划时调用，重置：
 * - 目标到达标志、等待标志、后退模式等逻辑状态
 * - 横向误差统计量（最大值、平均值等）
 * - 角速度平滑相关滤波器与历史记录
 * - 位置滤波器和角速度滤波器参数
 */
void RPPController::resetControllerState()
{
  back_follow_ = false;
  waiting_ = true;
  goal_reached_ = false;

  start_print = false;
  stop_print = false;

  // 重置误差统计
  max_error_ = 0.0;
  avg_error_ = 0.0;
  error_sum_ = 0.0;
  error_count_ = 0;
  current_lateral_error_ = 0.0;

  // 重置滤波器
  second_order_filter_.reset();
  previous_angular_vel_ = 0.0;
  angular_vel_history_.clear();

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

  last_closest_idx = 0;
}

/**
 * @brief 重置圆形路径相关状态变量
 *
 * 清空累计转角、上一帧航向角等数据，使得每次设置新的
 * 圆形路径时不会受上一条圆形路径跟踪历史的影响。
 */
void RPPController::resetCirclePathState()
{
  last_yaw_initialized_ = false;
  last_yaw_ = 0.0;
  accumulated_angle_ = 0.0;
  angle_debug_counter_ = 0;
}

/**
 * @brief 根据圆弧半径调整速度和前瞻参数
 *
 * 半径越小曲率越大，对速度和前瞻距离要求越保守，/
 * 防止在小半径圆弧上出现过大的离心和横向误差。
 *
 * @param radius 圆弧半径
 */
void RPPController::adjustSpeedForRadius(double radius)
{
  if (radius < 0.5)
  {
    min_lookahead_dist_ = 0.15;
    max_lookahead_dist_ = 0.15;
    max_v_ = 0.08;
    min_v_ = 0.08;
    linear_speed_ = 0.08;
  }
  else if (radius < 0.8)
  {
    min_lookahead_dist_ = 0.17;
    max_lookahead_dist_ = 0.17;
    max_v_ = 0.10;
    min_v_ = 0.10;
    linear_speed_ = 0.10;
  }
  else if (radius < 1.2)
  {
    min_lookahead_dist_ = 0.21;
    max_lookahead_dist_ = 0.21;
    max_v_ = 0.11;
    min_v_ = 0.11;
    linear_speed_ = 0.11;
  }
  else
  {
    min_lookahead_dist_ = 0.17;
    max_lookahead_dist_ = 0.17;
    max_v_ = 0.11;
    min_v_ = 0.11;
    linear_speed_ = 0.11;
  }
}

/**
 * @brief 生成圆形路径（以当前机器人位姿为切入点）
 *
 * 传入的 robot_pose 被视为已经在圆周上的“切入点”，不再根据
 * 机器人与圆心的几何关系重新计算切入点坐标，仅基于该点生成
 * 圆周轨迹，并计算对应的切线方向用于航向预对准。
 *
 * @param circle_center_x 圆心 x 坐标
 * @param circle_center_y 圆心 y 坐标
 * @param circle_radius 圆半径
 * @param robot_pose 当前机器人位姿（即圆周切入点）
 * @return 包含切入点和圆周段的路径
 */
nav_msgs::msg::Path RPPController::generateCirclePath(double circle_center_x, double circle_center_y,
                                                       double circle_radius,
                                                       const geometry_msgs::msg::PoseStamped& start_pose)
{
  nav_msgs::msg::Path circle_path;
  circle_path.header.frame_id = "world";
  circle_path.header.stamp = this->now();

  // 直接使用传入的机器人位姿作为圆周切入点
  geometry_msgs::msg::PoseStamped entry_pose = start_pose;

  // 缓存切入点坐标，便于后续调试或扩展使用
  circle_entry_x_ = entry_pose.pose.position.x;
  circle_entry_y_ = entry_pose.pose.position.y;

  // 计算切入点处圆周切线方向，用于设置目标航向角
  double tangent_x = -(entry_pose.pose.position.y - circle_center_y);
  double tangent_y = (entry_pose.pose.position.x - circle_center_x);
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

  // 添加切入点
  circle_path.poses.push_back(entry_pose);

  // 生成圆周路径
  int num_circle_points = 1500;
  double start_angle = atan2(entry_pose.pose.position.y - circle_center_y,
                             entry_pose.pose.position.x - circle_center_x);
  double total_angle = circle_total_angle;

  for (int i = 20; i <= num_circle_points; i++)
  {
    double angle = start_angle + i * (total_angle / num_circle_points);

    geometry_msgs::msg::PoseStamped circle_pose;
    circle_pose.pose.position.x = circle_center_x + circle_radius * cos(angle);
    circle_pose.pose.position.y = circle_center_y + circle_radius * sin(angle);
    circle_pose.pose.position.z = start_pose.pose.position.z;

    double tangent_direction = angle + M_PI / 2;
    tf2::Quaternion circle_q;
    circle_q.setRPY(0, 0, tangent_direction);
    circle_pose.pose.orientation = tf2::toMsg(circle_q);

    circle_path.poses.push_back(circle_pose);
  }

  return circle_path;
}

/**
 * @brief 对原始路径进行插值和平滑
 *
 * 对相邻路径点距离大于阈值的段进行线性插值，插入若干中间点，
 * 并根据段的方向重建中间点的朝向，从而生成更为细腻、连续的
 * 路径，提高纯跟踪算法的控制稳定性。
 *
 * @param orig_path 原始路径
 * @return 平滑和细分后的路径
 */
nav_msgs::msg::Path RPPController::smoothAndSubdividePath(const nav_msgs::msg::Path& orig_path)
{
  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = orig_path.header;
  smoothed_path.poses.push_back(orig_path.poses.front());

  for (size_t i = 0; i < orig_path.poses.size() - 1; ++i)
  {
    const auto& current = orig_path.poses[i];
    const auto& next = orig_path.poses[i + 1];

    double segment_length = std::hypot(next.pose.position.x - current.pose.position.x,
                                       next.pose.position.y - current.pose.position.y);

    if (segment_length > 0.003)
    {
      int num_subdivisions = std::ceil(segment_length / 0.002);

      for (int j = 1; j < num_subdivisions; ++j)
      {
        double ratio = static_cast<double>(j) / num_subdivisions;

        geometry_msgs::msg::PoseStamped intermediate_pose;
        intermediate_pose.header = current.header;
        intermediate_pose.pose.position.x = current.pose.position.x +
            ratio * (next.pose.position.x - current.pose.position.x);
        intermediate_pose.pose.position.y = current.pose.position.y +
            ratio * (next.pose.position.y - current.pose.position.y);
        intermediate_pose.pose.position.z = current.pose.position.z +
            ratio * (next.pose.position.z - current.pose.position.z);

        double yaw = atan2(next.pose.position.y - current.pose.position.y,
                           next.pose.position.x - current.pose.position.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        intermediate_pose.pose.orientation = tf2::toMsg(q);

        smoothed_path.poses.push_back(intermediate_pose);
      }
    }

    if (i < orig_path.poses.size() - 2)
    {
      smoothed_path.poses.push_back(next);
    }
  }

  smoothed_path.poses.push_back(orig_path.poses.back());
  return smoothed_path;
}

/**
 * @brief 检查并清理路径中的无效点
 *
 * 将位置坐标包含 NaN/Inf 的路径点剔除；若清理后路径为空，
 * 直接返回失败。若仅部分点无效，会输出警告并替换为清理后的路径。
 *
 * @return 清理成功且路径非空返回 true
 */
bool RPPController::validateAndCleanPath()
{
  std::vector<geometry_msgs::msg::PoseStamped> valid_poses;
  valid_poses.reserve(global_plan_.poses.size());

  for (const auto& pose : global_plan_.poses)
  {
    if (std::isfinite(pose.pose.position.x) &&
        std::isfinite(pose.pose.position.y) &&
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

  if (valid_poses.size() != global_plan_.poses.size())
  {
    RCLCPP_WARN(get_logger(), "路径包含无效点，已清理: %zu -> %zu",
                global_plan_.poses.size(), valid_poses.size());
    global_plan_.poses = valid_poses;
  }

  return true;
}

/**
 * @brief 计算路径长度和目标点信息
 *
 * 遍历路径点累加弧长得到路径总长度，并保存目标点的
 * 位置与朝向，为后续目标判定和调试输出提供信息。
 */
void RPPController::calculatePathInfo()
{
  path_length_ = 0.0;
  for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i)
  {
    path_length_ += std::hypot(global_plan_.poses[i + 1].pose.position.x - global_plan_.poses[i].pose.position.x,
                               global_plan_.poses[i + 1].pose.position.y - global_plan_.poses[i].pose.position.y);
  }

  remaining_distance_ = path_length_;
  traversed_distance_ = 0.0;

  const auto& goal_pose = global_plan_.poses.back();
  goal_x_ = goal_pose.pose.position.x;
  goal_y_ = goal_pose.pose.position.y;
  goal_theta_ = tf2::getYaw(goal_pose.pose.orientation);
  current_curvature_ = 0.0;

  RCLCPP_INFO(get_logger(), "路径长度: %.3fm, 目标: (%.3f, %.3f, %.3f)",
              path_length_, goal_x_, goal_y_, goal_theta_);
}

/**
 * @brief 初始化速度指令消息
 *
 * 将线速度与角速度置零，并设置当前时间和基座坐标系，
 * 作为后续控制计算的基础。
 */
void RPPController::initializeCommandVel(geometry_msgs::msg::TwistStamped& cmd_vel)
{
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = "base_link";
}

/**
 * @brief 使用多级滤波器对机器人位姿进行平滑
 *
 * 先使用 Hampel 滤波抑制异常点，再通过 Savitzky-Golay 滤波
 * 平滑曲线，最后可选地使用四阶低通滤波器进一步抑制高频噪声。
 *
 * @param robot_pose 原始机器人位姿
 * @return 滤波后的位姿（仅平滑位置，姿态保持不变）
 */
geometry_msgs::msg::PoseStamped RPPController::filterRobotPose(const geometry_msgs::msg::PoseStamped& robot_pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header = robot_pose.header;
  current_pose.pose.orientation = robot_pose.pose.orientation;

  double filtered_x = h_x_filter.filter(robot_pose.pose.position.x);
  double filtered_y = h_y_filter.filter(robot_pose.pose.position.y);

  filtered_x = sg_x_filter_.filter(filtered_x);
  filtered_y = sg_y_filter_.filter(filtered_y);

  if (pos_use_biquad_cascade_filter_)
  {
    filtered_x = pos_x_filter_.filter(filtered_x);
    filtered_y = pos_y_filter_.filter(filtered_y);
  }

  current_pose.pose.position.x = filtered_x;
  current_pose.pose.position.y = filtered_y;

  return current_pose;
}

/**
 * @brief 在后退模式下对位姿进行等效变换
 *
 * 通过在当前航向上加 π，将后退跟踪问题转化为等效的前进问题，
 * 从而复用相同的控制逻辑，仅在最终输出时对线速度取反。
 *
 * @param pose 原始机器人位姿
 * @return 等效后的机器人位姿
 */
geometry_msgs::msg::PoseStamped RPPController::adjustPoseForBackward(const geometry_msgs::msg::PoseStamped& pose)
{
  geometry_msgs::msg::PoseStamped adjusted_pose = pose;
  double current_yaw = tf2::getYaw(pose.pose.orientation);
  double modified_yaw = atan2(sin(current_yaw + M_PI), cos(current_yaw + M_PI));

  tf2::Quaternion q;
  q.setRPY(0, 0, modified_yaw);
  adjusted_pose.pose.orientation = tf2::toMsg(q);

  return adjusted_pose;
}

/**
 * @brief 判断位姿的位置信息是否有效
 *
 * 当前实现主要检查 x、y 坐标为有限数值，避免无效数据
 * 进入后续几何计算。
 *
 * @param pose 待检查的位姿
 * @return 合法返回 true
 */
bool RPPController::isValidPose(const geometry_msgs::msg::PoseStamped& pose)
{
  return std::isfinite(pose.pose.position.x) && std::isfinite(pose.pose.position.y);
}

/**
 * @brief 检查直线路径/通用路径是否已经到达目标
 *
 * 计算当前位姿与路径末端之间的距离，更新剩余距离与已行驶距离，
 * 并在距离小于阈值时调用 setGoalReachedState() 设置停止状态。
 * 若当前为圆形路径，则交由专门的圆形路径结束判定函数处理。
 *
 * @param current_pose 当前机器人位姿
 * @param cmd_vel 当前速度指令（可能被置零）
 * @return 已到达目标返回 true
 */
bool RPPController::checkGoalReached(const geometry_msgs::msg::PoseStamped& current_pose,
                                     geometry_msgs::msg::TwistStamped& cmd_vel)
{
  auto goal_pose = global_plan_.poses.back();
  double distance_to_goal = std::hypot(current_pose.pose.position.x - goal_pose.pose.position.x,
                                       current_pose.pose.position.y - goal_pose.pose.position.y);

  remaining_distance_ = distance_to_goal;
  traversed_distance_ = path_length_ - remaining_distance_;

  if (is_circle_path)
  {
    return checkCirclePathGoalReached(current_pose, cmd_vel);
  }
  else
  {
    if (distance_to_goal < goal_dist_tol_)
    {
      setGoalReachedState(cmd_vel);
      RCLCPP_INFO(get_logger(), "目标已达到 - 最终误差: %.4fm", remaining_distance_);
      return true;
    }
  }

  return false;
}

/**
 * @brief 针对圆形路径的到达判定
 *
 * 通过累积每一控制周期的航向角变化量，计算机器人沿圆弧行驶的
 * 总转角，当累计角度达到设定的 circle_total_angle 附近时，
 * 认为圆形路径跟随完成，并设置目标到达状态。
 *
 * @param current_pose 当前机器人位姿
 * @param cmd_vel 当前速度指令（可能被置零）
 * @return 完成预定圆弧段返回 true
 */
bool RPPController::checkCirclePathGoalReached(const geometry_msgs::msg::PoseStamped& current_pose,
                                               geometry_msgs::msg::TwistStamped& cmd_vel)
{
  tf2::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z, current_pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  if (!last_yaw_initialized_)
  {
    last_yaw_ = yaw;
    last_yaw_initialized_ = true;
    accumulated_angle_ = 0.0;
    RCLCPP_INFO(get_logger(), "圆形路径跟踪开始，初始航向角: %.2f", yaw);
  }
  else
  {
    double delta_yaw = yaw - last_yaw_;

    if (delta_yaw > M_PI)
    {
      delta_yaw -= 2.0 * M_PI;
    }
    else if (delta_yaw < -M_PI)
    {
      delta_yaw += 2.0 * M_PI;
    }

    if (delta_yaw > 0)
    {
      accumulated_angle_ += delta_yaw;
    }

    last_yaw_ = yaw;

    if (accumulated_angle_ > 0.2 * M_PI)
    {
      // 标记开始打印
      start_print = true;
      stop_print = false;
    }
    
    if (accumulated_angle_ >= (circle_total_angle - 0.4 * M_PI))
    {

      // 标记结束打印
      start_print = false;
      stop_print = true;

      setGoalReachedState(cmd_vel);
      RCLCPP_INFO(get_logger(), "圆形路径完成 - 累计角度: %.4f rad (%.2f°)",
                  accumulated_angle_, accumulated_angle_ * 180.0 / M_PI);

      last_yaw_initialized_ = false;
      accumulated_angle_ = 0.0;
      return true;
    }
  }

  return false;
}

/**
 * @brief 设置控制器到达目标后的状态
 *
 * 将内部标志 goal_reached_ 置位，并将线速度与角速度清零，
 * 同时输出一次整体跟踪误差统计信息。
 *
 * @param cmd_vel 当前速度指令
 */
void RPPController::setGoalReachedState(geometry_msgs::msg::TwistStamped& cmd_vel)
{
  goal_reached_ = true;
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = "base_link";

  if (error_count_ > 0)
  {
    avg_error_ = error_sum_ / error_count_;
    RCLCPP_INFO(get_logger(), "跟踪性能统计 - 最大误差: %.4f m, 平均误差: %.4f m", max_error_, avg_error_);
  }
}

/**
 * @brief 当前瞻点无效时使用的备用前瞻点
 *
 * 简单地选择裁剪后路径中的第二个点（或第一个点），
 * 作为保底的前瞻点，防止控制流程因数值异常而中断。
 *
 * @param pruned_plan 裁剪后的路径
 * @return 备用前瞻点位姿
 */
geometry_msgs::msg::PoseStamped RPPController::getFallbackLookaheadPoint(
    const std::vector<geometry_msgs::msg::PoseStamped>& pruned_plan)
{
  if (!pruned_plan.empty())
  {
    return pruned_plan[std::min(size_t(1), pruned_plan.size() - 1)];
  }
  return geometry_msgs::msg::PoseStamped();
}

/**
 * @brief 计算机器人到前瞻点的角度误差
 *
 * 将前瞻点转换为 PointStamped，复用 dphi() 接口，
 * 得到当前需要转向的角度偏差。
 *
 * @param lookahead_pose 前瞻点位姿
 * @param current_pose 当前机器人位姿
 * @return 角度误差（弧度）
 */
double RPPController::computeAngleToLookahead(const geometry_msgs::msg::PoseStamped& lookahead_pose,
                                               const geometry_msgs::msg::PoseStamped& current_pose)
{
  geometry_msgs::msg::PointStamped lookahead_pt;
  lookahead_pt.header = lookahead_pose.header;
  lookahead_pt.point = lookahead_pose.pose.position;

  double angle = dphi(lookahead_pt, current_pose);
  return std::isfinite(angle) ? angle : 0.0;
}

/**
 * @brief 根据 Pure Pursuit 几何关系计算曲率
 *
 * 使用公式 k = 2 * sin(alpha) / L，其中 alpha 为角度误差，
 * L 为前瞻距离。
 *
 * @param angle_to_lookahead 机器人到前瞻点的角度误差
 * @param lookahead_distance 前瞻距离
 * @return 路径曲率
 */
double RPPController::computeCurvature(double angle_to_lookahead, double lookahead_distance)
{
  if (std::abs(lookahead_distance) > 1e-6)
  {
    double curvature = 2.0 * sin(angle_to_lookahead) / lookahead_distance;
    return std::isfinite(curvature) ? curvature : 0.0;
  }
  return 0.0;
}

/**
 * @brief 使用几何关系估算横向误差
 *
 * 在小角度近似下，可将横向误差视为 L * sin(alpha)，
 * 其中 L 为前瞻距离，alpha 为角度误差。
 *
 * @param angle_to_lookahead 角度误差
 * @param lookahead_distance 前瞻距离
 * @return 横向误差
 */
double RPPController::computeLateralError(double angle_to_lookahead, double lookahead_distance)
{
  if (std::abs(lookahead_distance) > 1e-6 && std::isfinite(angle_to_lookahead))
  {
    double error = lookahead_distance * sin(angle_to_lookahead);
    return std::isfinite(error) ? error : 0.0;
  }
  return 0.0;
}

/**
 * @brief 更新横向误差的统计信息
 *
 * 利用当前横向误差更新误差累积和计数，维护最大误差与
 * 平均误差，用于最终性能统计。
 */
void RPPController::updateErrorStatistics()
{
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
}

/**
 * @brief 计算期望角速度（综合几何关系与圆形路径约束）
 *
 * 先根据 Pure Pursuit 几何关系计算原始角速度，再在圆形路径
 * 模式下将其约束在基准角速度的附近，并通过 smoothAngularVelocity()
 * 做进一步平滑，避免突然的角度变化。
 *
 * @param desired_velocity 期望线速度
 * @param curvature 当前路径曲率
 * @param current_angular_vel 当前角速度
 * @param lookahead_distance 前瞻距离
 * @param angle_to_lookahead 角度误差
 * @param dt 控制周期
 * @param filter_reset 是否重置平滑滤波器
 * @return 期望角速度
 */
double RPPController::computeDesiredAngularVelocity(double desired_velocity, double curvature,
                                                     double current_angular_vel, double lookahead_distance,
                                                     double angle_to_lookahead, double dt, bool& filter_reset)
{
  // 若期望线速度或曲率为非有限值（NaN/Inf），直接返回 0，避免后续计算产生异常
  if (!std::isfinite(desired_velocity) || !std::isfinite(curvature))
  {
    return 0.0;
  }

  // 标准 Pure Pursuit 模型的角速度：w = v * k
  // 其中 v 为线速度，k 为路径曲率（1/半径），pp_angular_velocity 作为未调节的基础角速度
  double pp_angular_velocity = desired_velocity * curvature;
  // 初始时将期望角速度设为 PP 角速度，后续在圆形路径下可能会根据累计角度做修正
  double desired_angular_velocity = pp_angular_velocity;

  // 仅在当前路径为「圆形路径」时，才启用基于累计转角的角速度调节逻辑
  if (is_circle_path)
  {
    // accumulated_angle_ 为机器人沿圆形路径已经行驶的弧度总和
    // 当累计角度尚未达到 (start_deviation_factor_ - 0.1)π 时，认为处于起始过渡区，不做半径偏差修正
    if (accumulated_angle_ >= ((start_deviation_factor_ - 0.1) * M_PI))
    {
      // 计算当前 PP 角速度与圆形路径基准角速度之间的差值
      // baseline_angular_velocity_for_circle_ = min_v_ / circle_radius
      double angular_velocity_delta = desired_angular_velocity - baseline_angular_velocity_for_circle_;
      // 默认允许的角速度误差比例（相对基准角速度的百分比）
      double error_ratio = 0.05;

      // 在圆弧的起始区间内：使用配置的 deviation_rate_ 作为误差比例
      // 这样可以在刚进入圆形路径时给 PP 算法更大的调节空间，加快对预期半径的收敛
      if (accumulated_angle_ < (start_deviation_factor_ * M_PI))
      {
        error_ratio = deviation_rate_;
      }
      // 在圆弧的末尾区间内：同样使用 deviation_rate_，提升对终点段的误差修正能力
      if (accumulated_angle_ > (circle_total_angle - end_deviation_factor_ * M_PI))
      {
        error_ratio = deviation_rate_;
      }

      // 按照误差比例限制当前允许的最大角速度偏差
      // 避免因瞬时曲率异常导致角速度变化过大，引起车辆姿态剧烈抖动
      double max_angular_delta = baseline_angular_velocity_for_circle_ * error_ratio;
      if (std::abs(angular_velocity_delta) > max_angular_delta)
      {
        // 将实际偏差裁剪在 [-max_angular_delta, max_angular_delta] 范围内，并保留原符号
        angular_velocity_delta = std::copysign(max_angular_delta, angular_velocity_delta);
      }

      // 在基准圆形角速度的基础上叠加有限偏差，得到本次期望角速度
      desired_angular_velocity = baseline_angular_velocity_for_circle_ + angular_velocity_delta;
      // 对期望角速度做平滑处理，减少控制量的突变
      // filter_reset 由外部传入并在此分支结束后置为 false，表示滤波器可以延续历史状态
      desired_angular_velocity = smoothAngularVelocity(current_angular_vel, desired_angular_velocity,
                                                       lookahead_distance, angle_to_lookahead, dt, filter_reset);
      filter_reset = false;
    }
    else
    {
      // 累计角度仍处于起始过渡阶段：此时不使用基准圆形角速度，只对原始 PP 角速度进行平滑
      // 通过传入 reset=false（固定值），让角速度滤波器从稳定状态开始，避免一开始就继承旧状态
      desired_angular_velocity = smoothAngularVelocity(current_angular_vel, pp_angular_velocity,
                                                       lookahead_distance, angle_to_lookahead, dt, false);
      // 将 filter_reset 置为 true，提示外部在下一阶段切换时可重新初始化滤波器状态
      filter_reset = true;
    }
  }

  // 最终返回有限的角速度结果，如果出现非有限值则安全地回退为 0
  return std::isfinite(desired_angular_velocity) ? desired_angular_velocity : 0.0;
}

/**
 * @brief 生成最终的速度控制指令
 *
 * 通过 linearRegularization() 和角速度增益对期望速度进行整形，
 * 同时在后退模式下对线速度取反，并进行有限性检查，保证输出
 * 速度始终在合理范围内。
 *
 * @param cmd_vel 输出的速度指令
 * @param current_velocity 当前机器人速度
 * @param desired_velocity 期望线速度
 * @param desired_angular_velocity 期望角速度
 */
void RPPController::generateVelocityCommand(geometry_msgs::msg::TwistStamped& cmd_vel,
                                             const geometry_msgs::msg::Twist& current_velocity,
                                             double desired_velocity, double desired_angular_velocity)
{
  cmd_vel.twist.linear.x = linearRegularization(current_velocity.linear.x, desired_velocity);
  desired_velocity_ = desired_velocity;

  cmd_vel.twist.angular.z = desired_angular_velocity;

  RCLCPP_INFO(get_logger(), "raw=%.3f, gain=%.3f, base=%.3f",
               desired_angular_velocity, cmd_vel.twist.angular.z, baseline_angular_velocity_for_circle_);

  if (back_follow_ && !is_circle_path)
  {
    cmd_vel.twist.linear.x = -cmd_vel.twist.linear.x;
  }

  // 安全检查
  if (!std::isfinite(cmd_vel.twist.linear.x))
  {
    cmd_vel.twist.linear.x = min_v_;
  }
  if (!std::isfinite(cmd_vel.twist.angular.z))
  {
    cmd_vel.twist.angular.z = 0.0;
  }

  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = "base_link";
}

/**
 * @brief 按固定时间间隔更新栅格图
 *
 * 为减少磁盘写入频率，仅当距离上次更新超过 1s 时，
 * 才刷新机器人和前瞻点在图中的位置并保存栅格图。
 *
 * @param current_pose 当前机器人位姿
 * @param lookahead_pose 当前前瞻点位姿
 */
void RPPController::updateGridMapIfNeeded(const geometry_msgs::msg::PoseStamped& current_pose,
                                           const geometry_msgs::msg::PoseStamped& lookahead_pose)
{
  if (!enable_grid_map_)
  {
    return;
  }

  auto current_time = this->now();
  if ((current_time - last_grid_update_time_).seconds() > 1.0)
  {
    drawRobotOnGrid(current_pose);
    if (isValidPose(lookahead_pose))
    {
      drawLookaheadPointOnGrid(lookahead_pose.pose.position);
    }
    saveGridMap();
    last_grid_update_time_ = current_time;
  }
}

/**
 * @brief 检查一次控制循环的执行时间
 *
 * 若计算耗时接近控制周期（> 90%），输出警告信息，
 * 方便定位性能瓶颈。
 *
 * @param start_time 本次计算开始时间
 */
void RPPController::checkComputationTime(const rclcpp::Time& start_time)
{
  auto duration = this->now() - start_time;
  if (duration.seconds() > d_t_ * 0.9)
  {
    RCLCPP_WARN(get_logger(), "控制循环耗时过长: %.3f ms", duration.seconds() * 1000.0);
  }
}

/**
 * @brief 判断给定像素点是否在栅格图有效区域内
 *
 * @param pt 像素坐标
 * @return 在图像范围内返回 true
 */
bool RPPController::isPointInGrid(const cv::Point& pt)
{
  return pt.x >= 0 && pt.x < grid_map_.cols && pt.y >= 0 && pt.y < grid_map_.rows;
}

}  // namespace follow_controller
}  // namespace xline
