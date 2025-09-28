#include "xline_follow_controller/line_follow_controller.hpp"
#include "xline_follow_controller/yaml_parser.hpp"
#include <angles/angles.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <thread>
#include <functional>

namespace xline
{
namespace follow_controller
{

LineFollowController::LineFollowController()
  : current_state_(ControlState::IDLE)
  , received_plan_(false)
  , goal_reached_(false)
  , back_follow_(false)
  , m_work_state_(true)
  , short_path_(false)
  , reset_required_(false)
  , debug_(true)
  , decel_phase_entered_(false)
  , current_waypoint_index_(0)
  , path_length_(0.0)
  , robot_yaw_(0.0)
  , current_linear_speed_(0.0)
  , current_angular_speed_(0.0)
  , prev_angular_velocity_(0.0)
  , last_yaw_error_(0.0)
  , wait_duration_(0.5)
  , waiting_(false)
  , angular_smoother_(2.0, 0.7)
  , angular_vel_hampel_filter_(5, 3.0)
  , imu_node_(nullptr)
  , imu_thread_running_(false)
{
  // 初始化默认参数
  initializeDefaultParameters();

  // 从配置文件更新参数
  updateParameters();

  // 初始化滤波器
  initializeFilters();

  // 初始化PID控制器
  pid_heading_controller_ = std::make_shared<PIDController>(angular_kp_, angular_ki_, angular_kd_, 0.2, -0.2);
  pid_heading_controller_->setIntegralLimits(-0.05, 0.05);

  // 初始化时间
  last_time_ = std::chrono::steady_clock::now();
  wait_start_time_ = std::chrono::steady_clock::now();

  // 初始化IMU订阅（使用内部节点,不依赖外部节点）
  initializeImuSubscription();

  // 初始化地形日志发布器
  terrain_log_topic_name_ = "/terrain_classification_log";
  initializeTerrainLogPublisher();

  // LOG_INFO("LineFollowController 初始化完成");
}

LineFollowController::~LineFollowController()
{
  // 停止IMU线程
  imu_thread_running_ = false;
  if (imu_thread_.joinable())
  {
    imu_thread_.join();
    LOG_INFO("IMU独立线程已停止");
  }

  // 清理IMU订阅和节点
  if (imu_subscription_)
  {
    imu_subscription_.reset();
  }

  if (imu_node_)
  {
    imu_node_.reset();
    LOG_INFO("IMU专用内部节点已清理");
  }

  // 清理运动状态检测订阅和节点
  if (cmd_vel_subscription_)
  {
    cmd_vel_subscription_.reset();
  }

  if (motion_node_)
  {
    motion_node_.reset();
    LOG_INFO("运动状态检测节点已清理");
  }

  // 清理地形日志发布器
  if (terrain_log_publisher_)
  {
    terrain_log_publisher_.reset();
    LOG_INFO("地形日志发布器已清理");
  }
}

void LineFollowController::initializeDefaultParameters()
{
  // 速度参数默认值
  max_linear_speed_ = 0.5;
  min_linear_speed_ = 0.15;
  m_walk_max_vel_ = 0.26;
  m_work_max_vel_ = 0.26;
  m_alignment_vel_ = 0.12;

  // 角速度参数默认值
  max_rotation_angular_vel_ = 0.8;
  min_rotation_angular_vel_ = 0.35;
  rotation_angular_factor_ = 0.65;
  rotation_angle_threshold_ = 0.1;
  rotation_angle_smooth_factor_ = 0.5;

  // 距离参数默认值
  waypoint_tolerance_ = 0.005;
  yaw_tolerance_ = 0.05;
  lookahead_distance_ = 0.25;
  deceleration_distance_ = 0.2;
  m_alignment_distance_ = 0.25;
  m_acce_distance_ = 0.3;

  // 控制参数默认值
  m_acce_factor_ = 0.0003;
  m_acceleration_factor_ = 5.0;
  m_deceleration_factor_ = 5.0;
  m_acceleration_sigmoid_center_ = 0.5;
  m_deceleration_sigmoid_center_ = 0.5;
  max_angular_acceleration_ = 0.4;

  // PID参数默认值（激进优化）
  angular_kp_ = 1.1;  // 从配置文件的激进优化值
  angular_ki_ = 0.0;
  angular_kd_ = 0.12;

  // 滤波器参数默认值
  m_window_size_ = 5;
  m_polynomial_order_ = 2;
  m_hampel_window_ = 5;
  m_hampel_k_ = 3.0;

  // 二阶平滑器参数默认值
  angular_smoother_freq_ = 2.0;
  angular_smoother_damping_ = 0.7;

  // 当前滤波参数默认值（将在updateParameters中从配置文件更新）
  current_alpha_ = 0.85;
  current_smoother_frequency_ = 15.0;
  current_smoother_damping_ = 0.95;
  alpha_ = current_alpha_;

  // IMU地形自适应控制参数 - 将在updateParameters中从配置文件读取
  // 这里只设置基本的初始状态,具体参数值从line.yaml统一读取

  // 初始化地形分析数据
  terrain_data_.current_type = TerrainType::SMOOTH;
  terrain_data_.confidence_score = 0.0;
  terrain_data_.angle_vel_dynamic_factor = 1.0;
  terrain_data_.line_vel_dynamic_factor = 1.0;
  terrain_data_.last_update = std::chrono::steady_clock::now();

  // 初始化IMU处理线程
  imu_thread_running_.store(false);

  // 初始化地形状态记忆数据（状态机核心数据结构）
  auto init_time = std::chrono::steady_clock::now();

  // === 地形状态记忆初始化（立即切换模式简化版）===
  state_memory_.previous_terrain = TerrainType::SMOOTH;
  state_memory_.last_terrain_change_time = init_time;
  state_memory_.terrain_change_count = 0;
  state_memory_.sudden_change_count = 0;

  // 初始化迟滞控制
  // 立即切换模式：无需初始化迟滞控制参数

  // 初始化水泥地面模式
  current_pitch_angle_ = 0.0;
  current_yaw_rate_ = 0.0;

  // 翻滚角数据初始化
  current_roll_angle_ = 0.0;   // 当前翻滚角
  filtered_roll_angle_ = 0.0;  // 滤波后翻滚角

  // 数据记录参数初始化
  data_log_base_path_ = "/home/daosn_robotics/zyq_ws/terrain_data/";  // 默认路径
  enable_detailed_logging_ = true;                                    // 默认启用详细日志
  enable_imu_terrain_logging_ = true;                                 // 默认启用IMU地形日志
  logging_frequency_ = 20.0;                                          // 默认20Hz记录频率
  last_log_time_ = std::chrono::steady_clock::now();
}

void LineFollowController::updateParameters()
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("xline_follow_controller");
  std::string config_file_path = package_share_directory + "/config/line.yaml";

  // 使用优化后的YamlParser
  xline::YamlParser::YamlParser parser(config_file_path);

  // 调试：检查YAML文件是否正确加载
  // LOG_INFO("YAML文件路径: {}", config_file_path);
  auto all_keys = parser.getAllKeys();
  // LOG_INFO("YAML解析的键数量: {}", all_keys.size());
  if (debug_)
  {
    // LOG_INFO("=== 完整YAML解析验证 ===");
    // parser.printKeysWithPrefix("debug");
    // parser.printKeysWithPrefix("control");
    // parser.printKeysWithPrefix("motion");
    // parser.printKeysWithPrefix("sensors");
    // parser.printKeysWithPrefix("terrain_control");
    // parser.printKeysWithPrefix("data_logging");
    // LOG_INFO("=== 验证完成 ===");
  }

  try
  {
    // --- 调试开关 ---
    debug_ = parser.getParameter<bool>("debug");

    // --- 线速度相关 ---
    min_linear_speed_ = parser.getParameter<double>("motion.velocity.limits.min");
    m_walk_max_vel_ = parser.getParameter<double>("motion.velocity.limits.walk_max");
    m_work_max_vel_ = parser.getParameter<double>("motion.velocity.limits.work_max");
    m_alignment_vel_ = parser.getParameter<double>("motion.velocity.limits.alignment");

    // --- 距离参数 ---
    m_alignment_distance_ = parser.getParameter<double>("motion.velocity.distances.alignment");
    m_acce_distance_ = parser.getParameter<double>("motion.velocity.distances.acceleration");
    deceleration_distance_ = parser.getParameter<double>("motion.velocity.distances.deceleration");
    lookahead_distance_ = parser.getParameter<double>("motion.velocity.distances.lookahead");
    waypoint_tolerance_ = parser.getParameter<double>("motion.velocity.distances.waypoint_tolerance");
    mini_path_distance_ = parser.getParameter<double>("motion.velocity.distances.mini_path");

    // --- 调速参数 ---
    m_acce_factor_ = parser.getParameter<double>("motion.velocity.tuning.acce_factor");
    m_acceleration_factor_ = parser.getParameter<double>("motion.velocity.tuning.acceleration_factor");
    m_deceleration_factor_ = parser.getParameter<double>("motion.velocity.tuning.deceleration_factor");
    m_acceleration_sigmoid_center_ = parser.getParameter<double>("motion.velocity.tuning.acceleration_sigmoid_center");
    m_deceleration_sigmoid_center_ = parser.getParameter<double>("motion.velocity.tuning.deceleration_sigmoid_center");


    // --- 角速度参数 ---
    min_rotation_angular_vel_ = parser.getParameter<double>("motion.angular.rotation.limits.min");
    max_rotation_angular_vel_ = parser.getParameter<double>("motion.angular.rotation.limits.max");
    rotation_angular_factor_ = parser.getParameter<double>("motion.angular.rotation.tuning.factor");
    rotation_angle_threshold_ = parser.getParameter<double>("motion.angular.rotation.tuning.angle_threshold");
    rotation_angle_smooth_factor_ = parser.getParameter<double>("motion.angular.rotation.tuning.smooth_factor");
    angular_following_hampel_window_ = parser.getParameter<int>("motion.angular.following.hampel.window_size");
    angular_following_hampel_k_ = parser.getParameter<double>("motion.angular.following.hampel.k_threshold");



    // --- PID 控制器 ---
    angular_kp_ = parser.getParameter<double>("control.pid.kp");
    angular_ki_ = parser.getParameter<double>("control.pid.ki");
    angular_kd_ = parser.getParameter<double>("control.pid.kd");
    if (pid_heading_controller_)
    {
      pid_heading_controller_->setGains(angular_kp_, angular_ki_, angular_kd_);
    }

    // --- 滤波参数 ---
    m_window_size_ = parser.getParameter<int>("sensors.position.filtering.savgol.window_size");
    m_polynomial_order_ = parser.getParameter<int>("sensors.position.filtering.savgol.polynomial_order");
    m_hampel_window_ = parser.getParameter<int>("sensors.position.filtering.hampel.window_size");
    m_hampel_k_ = parser.getParameter<double>("sensors.position.filtering.hampel.k_threshold");

    // --- IMU 地形自适应 ---
    enable_imu_terrain_adaptation_ = parser.getParameter<bool>("sensors.imu.terrain_adaptation.enabled");
    imu_window_size_ = parser.getParameter<double>("sensors.imu.terrain_adaptation.window_size");

    sudden_change_gradient_threshold_ =
        parser.getParameter<double>("sensors.imu.terrain_adaptation.thresholds.sudden_change_gradient");
    gentle_low_freq_energy_threshold_ =
        parser.getParameter<double>("sensors.imu.terrain_adaptation.thresholds.gentle_low_freq_energy");
    frequent_zero_crossing_threshold_ =
        parser.getParameter<double>("sensors.imu.terrain_adaptation.thresholds.frequent_zero_crossing");
    max_imu_history_size_ = static_cast<size_t>(imu_window_size_ * 200.0);

    // --- 地形控制参数 ---
    auto loadSingleTerrain = [&parser](const std::string& terrain_type) -> TerrainControlParams {
      TerrainControlParams params;
      std::string base = "terrain_control." + terrain_type + ".";
      params.cross_track_deadzone = parser.getParameter<double>(base + "cross_track_deadzone");
      params.yaw_deadzone = parser.getParameter<double>(base + "yaw_deadzone");
      params.max_angular_vel = parser.getParameter<double>(base + "max_angular_vel");
      params.max_angular_accel = parser.getParameter<double>(base + "max_angular_accel");
      params.suppression_factor = parser.getParameter<double>(base + "suppression_factor");
      params.current_heading_weight = parser.getParameter<double>(base + "heading.current_heading_weight");
      params.target_heading_weight = parser.getParameter<double>(base + "heading.target_heading_weight");
      // 新增滤波参数读取
      params.alpha = parser.getParameter<double>(base + "filtering.alpha");
      params.smoother_frequency = parser.getParameter<double>(base + "smoother.frequency");
      params.smoother_damping = parser.getParameter<double>(base + "smoother.damping");
      return params;
    };

    sudden_change_params_ = loadSingleTerrain("sudden_change");
    gentle_undulation_params_ = loadSingleTerrain("gentle_undulation");
    frequent_bumps_params_ = loadSingleTerrain("frequent_bumps");
    smooth_terrain_params_ = loadSingleTerrain("smooth_terrain");
    alignment_params_ = loadSingleTerrain("alignment");

    // 初始化当前滤波参数（默认使用平稳地形参数）
    current_alpha_ = smooth_terrain_params_.alpha;
    current_smoother_frequency_ = smooth_terrain_params_.smoother_frequency;
    current_smoother_damping_ = smooth_terrain_params_.smoother_damping;
    alpha_ = current_alpha_;  // 设置当前使用的alpha
    
    // 更新二阶平滑器参数
    angular_smoother_.setParameters(current_smoother_frequency_, current_smoother_damping_);

    // --- 地形控制配置（立即切换模式） ---
    terrain_config_.immediate_switch = parser.getParameter<bool>("terrain_control_mode.immediate_switch");

    // --- 数据记录参数 ---
    data_log_base_path_ = parser.getParameter<std::string>("data_logging.base_path");
    enable_detailed_logging_ = parser.getParameter<bool>("data_logging.detailed_logging");
    enable_imu_terrain_logging_ = parser.getParameter<bool>("data_logging.enabled");
    logging_frequency_ = parser.getParameter<double>("data_logging.frequency");

    // --- 虚拟位置跟踪参数 ---
    virtual_tracking_config_.enabled = parser.getParameter<bool>("sensors.position.virtual_tracking.enabled");
    virtual_tracking_config_.correction_gain = parser.getParameter<double>("sensors.position.virtual_tracking.correction_gain");
    virtual_tracking_config_.max_correction = parser.getParameter<double>("sensors.position.virtual_tracking.max_correction");
    virtual_tracking_config_.position_trust_threshold = parser.getParameter<double>("sensors.position.virtual_tracking.position_trust_threshold");
    virtual_tracking_config_.max_velocity_trust = parser.getParameter<double>("sensors.position.virtual_tracking.kinematics.max_velocity_trust");
    virtual_tracking_config_.angular_velocity_factor = parser.getParameter<double>("sensors.position.virtual_tracking.kinematics.angular_velocity_factor");
    virtual_tracking_config_.smooth_gain = parser.getParameter<double>("sensors.position.virtual_tracking.terrain_adaptive.smooth_gain");
    virtual_tracking_config_.rough_gain = parser.getParameter<double>("sensors.position.virtual_tracking.terrain_adaptive.rough_gain");
    virtual_tracking_config_.sudden_change_gain = parser.getParameter<double>("sensors.position.virtual_tracking.terrain_adaptive.sudden_change_gain");
    virtual_tracking_config_.debug_logging = parser.getParameter<bool>("sensors.position.virtual_tracking.debug.enable_logging");
    virtual_tracking_config_.log_frequency = parser.getParameter<int>("sensors.position.virtual_tracking.debug.log_frequency");

    // 应用虚拟跟踪器参数
    virtual_tracker_.setParameters(
        virtual_tracking_config_.correction_gain,
        virtual_tracking_config_.max_correction,
        virtual_tracking_config_.position_trust_threshold,
        virtual_tracking_config_.max_velocity_trust,
        virtual_tracking_config_.angular_velocity_factor
    );
    
    virtual_tracker_.setTerrainGains(
        virtual_tracking_config_.smooth_gain,
        virtual_tracking_config_.rough_gain,
        virtual_tracking_config_.sudden_change_gain
    );
    
    virtual_tracker_.setDebugLogging(
        virtual_tracking_config_.debug_logging,
        virtual_tracking_config_.log_frequency
    );

    // --- 迟滞控制已移除（立即切换模式） ---

    if (debug_ && enable_imu_terrain_adaptation_)
    {
      // LOG_INFO("IMU地形自适应控制已启用 - 窗口:{:.1f}s", imu_window_size_);
      // LOG_INFO("检测阈值: 突变>{:.3f}, 平缓>{:.2f}, 频繁>{:.1f}", sudden_change_gradient_threshold_,
      //          gentle_low_freq_energy_threshold_, frequent_zero_crossing_threshold_);
    }
  }
  catch (const std::exception& e)
  {
    LOG_ERROR("参数加载失败: {}", e.what());
    throw;
  }
}

void LineFollowController::initializeFilters()
{
  x_filter_.reset(m_window_size_, m_polynomial_order_);
  y_filter_.reset(m_window_size_, m_polynomial_order_);
  h_x_filter_.reset(m_hampel_window_, m_hampel_k_);
  h_y_filter_.reset(m_hampel_window_, m_hampel_k_);
  angular_vel_hampel_filter_.reset(angular_following_hampel_window_, angular_following_hampel_k_);  // 重置角速度Hampel滤波器
}

void LineFollowController::resetControllerState()
{
  current_state_ = ControlState::IDLE;
  goal_reached_ = false;
  back_follow_ = false;
  reset_required_ = false;
  waiting_ = false;
  current_waypoint_index_ = 0;
  current_linear_speed_ = 0.0;
  current_angular_speed_ = 0.0;
  last_yaw_error_ = 0.0;
  decel_phase_entered_ = false;

  prev_angular_velocity_ = 0.0;
  second_prev_angular_velocity_ = 0.0;
  prev_smoothed_angular_velocity_ = 0.0;
  angular_vel_history_.clear();
  // 重置PID控制器
  if (pid_heading_controller_)
  {
    pid_heading_controller_->reset();
  }

  // 重置二阶平滑器
  angular_smoother_.reset();

  // 重置时间
  last_time_ = std::chrono::steady_clock::now();

  // LOG_INFO("控制器状态已重置");
}

bool LineFollowController::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
  if (orig_global_plan.poses.empty())
  {
    LOG_ERROR("接收到空路径");
    return false;
  }

  // 启动IMU处理线程（如果尚未启动）
  startImuThread();

  // 存储路径信息
  global_plan_ = orig_global_plan;
  start_pose_ = global_plan_.poses.front();
  original_target_pose_ = global_plan_.poses.back();  // 保存原始目标位姿
  target_pose_ = global_plan_.poses.back();
  end_pose_ = target_pose_;

  // 计算原始路径长度
  double dx = target_pose_.pose.position.x - start_pose_.pose.position.x;
  double dy = target_pose_.pose.position.y - start_pose_.pose.position.y;
  original_path_length_ = std::sqrt(dx * dx + dy * dy);

  // 初始化调试数据
  if (debug_)
  {
    original_path_.clear();
    filtered_path_.clear();
    original_path_.push_back({ start_pose_.pose.position.x, start_pose_.pose.position.y });
    original_path_.push_back({ target_pose_.pose.position.x, target_pose_.pose.position.y });
    filtered_path_.push_back({ start_pose_.pose.position.x, start_pose_.pose.position.y });
    filtered_path_.push_back({ target_pose_.pose.position.x, target_pose_.pose.position.y });
  }

  // 延长路径
  extendPath();

  // 更新路径长度为延长后的长度
  dx = target_pose_.pose.position.x - start_pose_.pose.position.x;
  dy = target_pose_.pose.position.y - start_pose_.pose.position.y;
  path_length_ = std::sqrt(dx * dx + dy * dy);

  // 更新参数
  updateParameters();


  // 保存原始距离参数用于短路径判断 
  double original_alignment_distance = m_alignment_distance_;
  double original_deceleration_distance = deceleration_distance_;
  double original_acce_distance = m_acce_distance_;
  
  // 判断是否为短路径：基于原始参数的总距离，加上安全余量
  double total_required_distance = original_alignment_distance + original_deceleration_distance + original_acce_distance;
  double safety_margin = std::max(0.1, total_required_distance * 0.1);  // 动态安全余量
  short_path_ = (path_length_ < (total_required_distance + safety_margin));
  if (short_path_)
  {
    if (path_length_ < 0.1)
    {
      // 极短路径：几乎不加速，主要是对齐和停止
      m_alignment_distance_ = path_length_ * 0.6;
      deceleration_distance_ = path_length_ * 0.4;
      m_acce_distance_ = 0;
    }
    else if (path_length_ < 0.2)
    {
      // 很短路径：简化为对齐+减速
      m_alignment_distance_ = path_length_ * 0.5;
      deceleration_distance_ = path_length_ * 0.5;
      m_acce_distance_ = 0;
    }
    else if (path_length_ < 0.4)
    {
      // 短路径：有少量加速空间
      m_alignment_distance_ = path_length_ * 0.3;
      deceleration_distance_ = path_length_ * 0.5;
      m_acce_distance_ = path_length_ * 0.2;
    }
    else
    {
      // 中等短路径：标准比例分配
      m_alignment_distance_ = path_length_ * 0.25;
      deceleration_distance_ = path_length_ * 0.5;
      m_acce_distance_ = path_length_ * 0.25;
    }
  }
  else
  {
    // ================================
    // 非短路径的动态优化策略
    // ================================
    double current_max_speed = m_work_state_ ? m_work_max_vel_ : m_walk_max_vel_;
    const double speed_threshold = 0.3;  // 速度优化阈值
    
    if (current_max_speed > speed_threshold)
    {
      // 计算理想的总控制距离（基于原始参数）
      double total_control_distance = original_alignment_distance + original_deceleration_distance + original_acce_distance;
      
      // 计算可用于高速段的距离
      double available_high_speed_distance = path_length_ - total_control_distance;
      
      if (available_high_speed_distance < path_length_ * 0.3)
      {
        // 高速段距离不足，需要降低最大速度
        double speed_reduction_factor = std::max(0.5, available_high_speed_distance / (path_length_ * 0.3));
        
        // 对最大速度进行衰减
        double optimized_max_speed = speed_threshold + (current_max_speed - speed_threshold) * speed_reduction_factor;
        
        if (m_work_state_)
        {
          m_work_max_vel_ = optimized_max_speed;
        }
        else
        {
          m_walk_max_vel_ = optimized_max_speed;
        }
        
        // 动态补偿减速距离（基于新的最大速度）
        double speed_ratio = optimized_max_speed / speed_threshold;
        deceleration_distance_ = original_deceleration_distance * std::pow(speed_ratio, 1.5);
        
        // 确保减速距离不超过路径的合理比例
        deceleration_distance_ = std::min(deceleration_distance_, path_length_ * 0.4);
        
        // 相应调整加速距离，保持对齐距离不变
        double remaining_distance = path_length_ - m_alignment_distance_ - deceleration_distance_;
        m_acce_distance_ = std::max(0.0, remaining_distance * 0.6);  // 60%用于加速，40%用于匀速
      }
      else if (current_max_speed > speed_threshold * 1.5)
      {
        // 速度过高，即使距离充足也要适当限制
        double conservative_factor = std::min(1.0, path_length_ / (total_control_distance * 2.0));
        double optimized_max_speed = speed_threshold * 1.5 * conservative_factor + 
                                   speed_threshold * 0.5 * (1.0 - conservative_factor);
        
        if (m_work_state_)
        {
          m_work_max_vel_ = optimized_max_speed;
        }
        else
        {
          m_walk_max_vel_ = optimized_max_speed;
        }
        
        // 按比例调整减速距离
        double speed_ratio = optimized_max_speed / speed_threshold;
        deceleration_distance_ = original_deceleration_distance * speed_ratio;
      }
    }
  }

  // 确保最小减速距离保护
  double current_max_speed = m_work_state_ ? m_work_max_vel_ : m_walk_max_vel_;
  double min_decel_distance = std::max(0.15, current_max_speed * 0.5);  // 基于速度的最小减速距离
  deceleration_distance_ = std::max(deceleration_distance_, min_decel_distance);

  // 重新初始化滤波器
  initializeFilters();
  // 重置控制器状态
  resetControllerState();
  
  // ================================
  // 重置虚拟位置跟踪器（新路径开始）
  // 清空上一次跟随流程的数据
  // ================================
  if (virtual_tracking_config_.enabled)
  {
    // 重置虚拟跟踪器状态
    virtual_tracker_.reset();
    
    // 重新应用配置参数
    virtual_tracker_.setParameters(
        virtual_tracking_config_.correction_gain,
        virtual_tracking_config_.max_correction,
        virtual_tracking_config_.position_trust_threshold,
        virtual_tracking_config_.max_velocity_trust,
        virtual_tracking_config_.angular_velocity_factor
    );
    
    virtual_tracker_.setTerrainGains(
        virtual_tracking_config_.smooth_gain,
        virtual_tracking_config_.rough_gain,
        virtual_tracking_config_.sudden_change_gain
    );
    
    virtual_tracker_.setDebugLogging(
        virtual_tracking_config_.debug_logging,
        virtual_tracking_config_.log_frequency
    );
  }
  
  received_plan_ = true;
  current_state_ = ControlState::ALIGNING_START;

  // 初始化任务ID和开始时间
  current_task_id_ = generateTaskId();
  task_start_time_ = std::chrono::steady_clock::now();

  return true;
}

bool LineFollowController::setPlan(const std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped>>& plan)
{
  if (!plan || plan->empty())
  {
    LOG_ERROR("接收到空路径或空指针");
    return false;
  }

  // 启动IMU处理线程（如果尚未启动）
  startImuThread();

  // 转换为nav_msgs::msg::Path格式
  nav_msgs::msg::Path new_global_plan;
  new_global_plan.header.frame_id = "world";

  for (const auto& pose : *plan)
  {
    new_global_plan.poses.push_back(pose);
  }

  end_pose_ = new_global_plan.poses.back();
  return setPlan(new_global_plan);
}

void LineFollowController::setSpeedLimit(const double& speed_limit)
{
  // updateParameters();
}

void LineFollowController::setWorkState(bool state)
{
  m_work_state_ = state;

  if (!m_work_state_)
  {
    // 判断是否需要后退
    bool should_back = shouldGoBackward(current_pose_.pose.position.x, current_pose_.pose.position.y,
                                        tf2::getYaw(current_pose_.pose.orientation), end_pose_.pose.position.x,
                                        end_pose_.pose.position.y, tf2::getYaw(end_pose_.pose.orientation));

    // 距离判断
    double distance = std::sqrt(std::pow(current_pose_.pose.position.x - end_pose_.pose.position.x, 2) +
                                std::pow(current_pose_.pose.position.y - end_pose_.pose.position.y, 2));

    if (should_back && distance < 1.5)
    {
      setBackFollow(true);
    }
  }
}

void LineFollowController::setBackFollow(bool back)
{
  back_follow_ = back;
  // LOG_INFO("设置后退模式: {}", back);
}

void LineFollowController::setPose(const geometry_msgs::msg::PoseStamped& pose)
{
  current_pose_ = pose;
}

bool LineFollowController::shouldGoBackward(double curr_x, double curr_y, double curr_yaw, double target_x,
                                            double target_y, double target_yaw)
{
  // 计算从当前点到目标点的向量及其角度
  double dx = target_x - curr_x;
  double dy = target_y - curr_y;
  double target_direction = std::atan2(dy, dx);

  // 正向行驶的转角计算
  double forward_initial_turn = normalizeAngle(target_direction - curr_yaw);
  double forward_final_turn = normalizeAngle(target_yaw - target_direction);
  double forward_total_turn = std::abs(forward_initial_turn) + std::abs(forward_final_turn);

  // 倒退行驶的转角计算
  double reverse_direction = normalizeAngle(target_direction + M_PI);
  double reverse_initial_turn = normalizeAngle(reverse_direction - curr_yaw);
  double reverse_final_turn = normalizeAngle(target_yaw - reverse_direction);
  double reverse_total_turn = std::abs(reverse_initial_turn) + std::abs(reverse_final_turn);

  // 选择总转角较小的方向
  return (forward_total_turn > reverse_total_turn);
}

double LineFollowController::normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}

geometry_msgs::msg::PoseStamped LineFollowController::getNextWaypoint(double robot_x, double robot_y)
{
  // 如果已到达最后一个路径点,返回终点
  if (current_waypoint_index_ >= global_plan_.poses.size() - 1)
  {
    return target_pose_;
  }

  // 查找前瞻距离内的路径点
  for (size_t i = current_waypoint_index_; i < global_plan_.poses.size(); ++i)
  {
    double dx = global_plan_.poses[i].pose.position.x - robot_x;
    double dy = global_plan_.poses[i].pose.position.y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance >= lookahead_distance_)
    {
      current_waypoint_index_ = i;
      return global_plan_.poses[i];
    }
  }

  // 如果没有找到满足条件的路径点,返回最后一个路径点
  current_waypoint_index_ = global_plan_.poses.size() - 1;
  return global_plan_.poses.back();
}

bool LineFollowController::isBeyondGoal(double robot_x, double robot_y)
{
  // 使用原始目标位置（延长前的位置）来判断是否到达终点
  // 计算起点到原始终点的向量
  double path_dx = original_target_pose_.pose.position.x - start_pose_.pose.position.x;
  double path_dy = original_target_pose_.pose.position.y - start_pose_.pose.position.y;
  double path_length = std::sqrt(path_dx * path_dx + path_dy * path_dy);

  // 如果路径长度为零,直接基于距离判断
  if (path_length == 0.0)
  {
    double distance_to_goal = std::sqrt(std::pow(robot_x - original_target_pose_.pose.position.x, 2) +
                                        std::pow(robot_y - original_target_pose_.pose.position.y, 2));
    return (distance_to_goal <= waypoint_tolerance_);
  }

  // 计算起点到机器人的向量
  double robot_dx = robot_x - start_pose_.pose.position.x;
  double robot_dy = robot_y - start_pose_.pose.position.y;

  // 计算投影长度
  double dot_product = robot_dx * path_dx + robot_dy * path_dy;
  double proj_length = dot_product / path_length;

  // 计算机器人到原始终点的距离
  double distance_to_goal = std::sqrt(std::pow(robot_x - original_target_pose_.pose.position.x, 2) +
                                      std::pow(robot_y - original_target_pose_.pose.position.y, 2));

  // 判断是否超过原始目标点
  return (proj_length > path_length) || (distance_to_goal <= waypoint_tolerance_);
}

bool LineFollowController::isAlignedWithTarget(double robot_yaw,
                                               const geometry_msgs::msg::Quaternion& target_orientation,
                                               bool is_backward)
{
  double target_yaw = tf2::getYaw(target_orientation);

  if (is_backward)
  {
    target_yaw += M_PI;
    target_yaw = std::atan2(std::sin(target_yaw), std::cos(target_yaw));
  }

  double yaw_error = angles::shortest_angular_distance(robot_yaw, target_yaw);

  // 检查是否越过目标航向
  bool has_crossed = (last_yaw_error_ > 0 && yaw_error < 0) || (last_yaw_error_ < 0 && yaw_error > 0);
  last_yaw_error_ = yaw_error;

  return (std::abs(yaw_error) < yaw_tolerance_) || has_crossed;
}

double LineFollowController::computeLinearSpeed(double distance_to_target, double distance_to_start)
{
  static double prev_speed = 0.0;
  double target_speed = 0.0;

  // 基于距离计算目标速度
  if (distance_to_target < deceleration_distance_)
  {
    // 减速阶段
    // 只在首次进入减速阶段时设置最大速度
    if (!decel_phase_entered_)
    {
      max_linear_speed_ = prev_speed;
      decel_phase_entered_ = true;
      prev_speed = 0.0;
    }

    const double precise_stop_distance = m_work_state_ ? 0.05 : 0.08;  // 精确停止距离

    if (distance_to_target <= precise_stop_distance)
    {
      target_speed = min_linear_speed_;
    }
    else
    {
      // 使用二次曲线实现平滑减速
      // double normalized_distance =
      //     (distance_to_target - precise_stop_distance) / (deceleration_distance_ - precise_stop_distance);
      // normalized_distance = std::max(0.0, std::min(1.0, normalized_distance));
      // double speed_factor = normalized_distance * normalized_distance;
      // target_speed = min_linear_speed_ + (max_linear_speed_ - min_linear_speed_) * speed_factor;


      // 使用 Sigmoid 函数实现平滑减速
      double normalized_distance =
          (distance_to_target - precise_stop_distance) / (deceleration_distance_ - precise_stop_distance);
      normalized_distance = std::max(0.0, std::min(1.0, normalized_distance));
      double sigmoid_value =
          1.0 / (1.0 + std::exp(-m_deceleration_factor_ * (normalized_distance - m_deceleration_sigmoid_center_)));
      target_speed = min_linear_speed_ + (max_linear_speed_ - min_linear_speed_) * sigmoid_value;

    }
  }
  else
  {
    // 加速阶段
    if (current_state_ == ControlState::ALIGNING_START || distance_to_start < m_alignment_distance_)
    {
      // 对齐阶段,使用固定的对齐速度
      target_speed = prev_speed + m_acce_factor_;
      max_linear_speed_ = m_alignment_vel_;
    }
    else
    {
      // 正常加速阶段
      // 根据工作状态设置最大速度
      max_linear_speed_ = m_work_state_ ? m_work_max_vel_ : m_walk_max_vel_;

      if (short_path_)
      {
        // 基于路径长度分层设置最大速度，与距离分配策略保持一致
        if (path_length_ < 0.1)
        {
          // 极短路径：使用最小速度
          max_linear_speed_ = min_linear_speed_;
        }
        else if (path_length_ < 0.2)
        {
          // 很短路径：稍高于最小速度
          max_linear_speed_ = min_linear_speed_ * 1.5;
        }
        else if (path_length_ < 0.4)
        {
          // 短路径：使用对齐速度
          max_linear_speed_ = m_alignment_vel_;
        }
        else
        {
          // 中等短路径：使用对齐速度的1.5倍
          max_linear_speed_ = m_alignment_vel_ * 1.5;
        }
        
        // 确保不超过标准最大速度的一半
        double standard_max_speed = m_work_state_ ? m_work_max_vel_ : m_walk_max_vel_;
        max_linear_speed_ = std::min(max_linear_speed_, standard_max_speed * 0.5);
      }

      // 使用sigmoid函数实现平滑加速
      double normalized_distance = (distance_to_start - m_alignment_distance_) / m_acce_distance_;
      normalized_distance = std::max(0.0, std::min(1.0, normalized_distance));

      double sigmoid_value =
          1.0 / (1.0 + std::exp(-m_acceleration_factor_ * (normalized_distance - m_acceleration_sigmoid_center_)));

      target_speed = m_alignment_vel_ + (max_linear_speed_ - m_alignment_vel_) * sigmoid_value;
    }

    // 限制最大速度
    target_speed = std::min(target_speed, max_linear_speed_);
  }

  // 应用地形动态因子（减速阶段不应用地形加速因子）
  if (distance_to_target >= deceleration_distance_)
  {
    target_speed *= terrain_data_.line_vel_dynamic_factor;
  }

  // 限制最小速度
  target_speed = std::max(target_speed, min_linear_speed_);

  // 速度变化率限制（防止急减速）
  static double last_update_time = 0.0;
  static std::chrono::steady_clock::time_point prev_time = std::chrono::steady_clock::now();
  
  auto current_time = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(current_time - prev_time).count();
  prev_time = current_time;
  
  if (dt > 0.001)  // 确保有效的时间间隔
  {
    double max_decel_rate = 0.5;  // m/s²，最大减速率
    double max_speed_change = max_decel_rate * dt;
    
    if (target_speed < prev_speed - max_speed_change)
    {
      target_speed = prev_speed - max_speed_change;
    }
  }

  // 平滑过渡
  if (distance_to_target > deceleration_distance_)
  {
    const double smooth_factor = 0.25;
    target_speed = prev_speed + smooth_factor * (target_speed - prev_speed);
  }
  
  prev_speed = target_speed;

  current_linear_speed_ = target_speed;

  if (path_length_ < mini_path_distance_)
  {
    current_linear_speed_ = min_linear_speed_;
  }
  return current_linear_speed_;
}

double LineFollowController::computeAngularVelocity(double yaw_error, double dt, double distance_to_target,
                                                    double distance_to_start, double linear_speed)
{
  // 标记未使用的参数以抑制编译警告
  (void)distance_to_target;
  (void)linear_speed;

  double deadzone_factor = 1.0;  // 默认无衰减

  // 基于IMU地形自适应的精度控制
  bool use_precision_control = false;
  double current_cross_track_deadzone = 0.0;                                // 当前跨轨死区
  double current_yaw_deadzone = 0.0;                                        // 当前偏航死区
  double current_max_angular_vel = smooth_terrain_params_.max_angular_vel;  // 当前最大角速度，默认使用平滑地形

  // use_precision_control直接由imu_terrain.enabled控制
  use_precision_control = enable_imu_terrain_adaptation_;

  // 根据距离选择适当的参数集（对齐阶段 vs 跟随阶段
  bool is_alignment_phase = (distance_to_start < m_alignment_distance_);

  if (use_precision_control && current_state_ == ControlState::FOLLOWING_PATH)
  {
    // 使用地形自适应参数,并根据阶段选择对应的参数集
    if (is_alignment_phase)
    {
      current_cross_track_deadzone = alignment_params_.cross_track_deadzone;
      current_yaw_deadzone = alignment_params_.yaw_deadzone;
      current_max_angular_vel = alignment_params_.max_angular_vel;
    }
    else
    {
      current_cross_track_deadzone = adaptive_params_.current_cross_track_deadzone;
      current_yaw_deadzone = adaptive_params_.current_yaw_deadzone;
      current_max_angular_vel = adaptive_params_.current_max_angular_vel;
    }
  }
  else
  {
    // 当use_precision_control为false时,使用平稳地形参数
    if (is_alignment_phase)
    {
      current_cross_track_deadzone = alignment_params_.cross_track_deadzone;
      current_yaw_deadzone = alignment_params_.yaw_deadzone;
      current_max_angular_vel = alignment_params_.max_angular_vel;
    }
    else
    {
      current_cross_track_deadzone = smooth_terrain_params_.cross_track_deadzone;
      current_yaw_deadzone = smooth_terrain_params_.yaw_deadzone;
      current_max_angular_vel = smooth_terrain_params_.max_angular_vel;
    }
  }

  // 使用当前最大角速度设置PID输出限制
  pid_heading_controller_->setOutputLimits(-current_max_angular_vel, current_max_angular_vel);

  // 计算横向偏差（无论use_precision_control是否为true都需要计算死区）
  double cross_track_error = computeCrossTrackError(current_pose_.pose.position.x, current_pose_.pose.position.y);

  // 使用软死区（渐进式衰减）而非硬死区,避免控制不连续
  // 计算死区衰减因子：误差越小,衰减越强
  double cross_track_factor = std::min(1.0, std::abs(cross_track_error) / current_cross_track_deadzone);
  double yaw_factor = std::min(1.0, std::abs(yaw_error) / current_yaw_deadzone);

  // 使用两个因子中的较大值,确保任一误差大时都有响应
  deadzone_factor = std::max(cross_track_factor, yaw_factor);

  // 如果完全在死区内,应用额外的衰减
  if (deadzone_factor < 1.0)
  {
    // 平滑过渡,避免突变
    deadzone_factor = std::pow(deadzone_factor, 2.0);  // 二次衰减曲线
  }

  // PID增益调整
  double d_adaptive_factor = 1.0;  // 默认不调整D项
  if (use_precision_control)
  {
    // 基于地形自适应调整D项增益
    pid_heading_controller_->setGains(angular_kp_, angular_ki_, angular_kd_ * d_adaptive_factor);
  }
  else
  {
    // 非地形自适应模式,恢复正常PID增益
    pid_heading_controller_->setGains(angular_kp_, angular_ki_, angular_kd_);
  }

  // 保存控制数据用于记录
  control_data_.deadzone_factor = deadzone_factor;
  control_data_.d_adaptive_factor = d_adaptive_factor;

  // 使用PID计算原始角速度
  double raw_angular_velocity = pid_heading_controller_->compute(yaw_error, dt);
  control_data_.pid_raw_output = raw_angular_velocity;

  // 计算并限制角加速度
  double angular_acceleration = (raw_angular_velocity - prev_angular_velocity_) / dt;
  control_data_.angular_acceleration = angular_acceleration;


  // 当前前使用的角加速度限制,默认使用平滑地形
  double current_max_angular_accel = smooth_terrain_params_.max_angular_accel;

  if (use_precision_control && current_state_ == ControlState::FOLLOWING_PATH)
  {
    // 使用地形自适应参数
    if (is_alignment_phase)
    {
      current_max_angular_accel = alignment_params_.max_angular_accel;
    }
    else
    {
      current_max_angular_accel = adaptive_params_.current_max_angular_accel;
    }
  }
  else
  {
    // 当use_precision_control为false时,使用平稳地形参数
    if (is_alignment_phase)
    {
      current_max_angular_accel = alignment_params_.max_angular_accel;
    }
    else
    {
      current_max_angular_accel = smooth_terrain_params_.max_angular_accel;
    }
  }

  control_data_.effective_max_angular_accel = current_max_angular_accel;
  control_data_.effective_max_angular_vel = current_max_angular_vel;

  if (debug_)
  {
    LOG_INFO(" ");
    LOG_INFO("角加速度: {:.5f}, 角加速度限制: {:.5f}", angular_acceleration,current_max_angular_accel);
    LOG_INFO("角速度: {:.5f}, 上一个角速度: {:.5f}", raw_angular_velocity,prev_angular_velocity_);
  }

  if (std::abs(angular_acceleration) > current_max_angular_accel)
  {
    angular_acceleration = (angular_acceleration > 0) ? current_max_angular_accel : -current_max_angular_accel;
    raw_angular_velocity = prev_angular_velocity_ + angular_acceleration * dt;
  }

  // 使用Hampel滤波器检测和修正角速度异常值
  // double outlier_filtered_vel = angular_vel_hampel_filter_.filter(raw_angular_velocity);

  // 二阶平滑器
  // double smoothed_angular_vel = angular_smoother_.filter(outlier_filtered_vel, dt);

  // ========== 以下代码已注释，保留备用 ==========
  double smoothed_angular_vel = 0.0;
  // 使用Hampel滤波器检测和修正角速度异常值
  smoothed_angular_vel = angular_vel_hampel_filter_.filter(raw_angular_velocity);
  if (debug_)
  {
    LOG_INFO("angular_vel_hampel_filter_: {:.5f}",smoothed_angular_vel);
  }
  // 低通滤波器
  smoothed_angular_vel = alpha_ * smoothed_angular_vel + (1 - alpha_) * prev_smoothed_angular_velocity_;
  prev_smoothed_angular_velocity_ = smoothed_angular_vel;
  if (debug_)
  {
    LOG_INFO("alpha_: {:.5f}",smoothed_angular_vel);
  }
  // 添加到历史记录
  // angular_vel_history_.push_back(lowpass_angular_vel);
  // if (angular_vel_history_.size() > angular_vel_history_size_)
  // {
  //   angular_vel_history_.pop_front();
  // }
  // for (double vel : angular_vel_history_)
  // {
  //   smoothed_angular_vel += vel;
  // }
  // // 移动平均
  // smoothed_angular_vel /= angular_vel_history_.size();


  // 二阶平滑器
  smoothed_angular_vel = angular_smoother_.filter(smoothed_angular_vel, dt);
  if (debug_)
  {
    LOG_INFO("angular_smoother_: {:.5f}",smoothed_angular_vel);
  }
  // 应用抑制因子和死区因子
  double current_suppression_factor;

  if (use_precision_control && current_state_ == ControlState::FOLLOWING_PATH)
  {
    // 地形自适应模式：使用自适应参数
    current_suppression_factor =
        is_alignment_phase ? alignment_params_.suppression_factor : adaptive_params_.current_suppression_factor;

    // 地形自适应控制：使用动态抑制因子
    smoothed_angular_vel *= current_suppression_factor * deadzone_factor;

    if (debug_)
    {
      // LOG_INFO("地形自适应控制 - 阶段:{}, 角速度限制:{:.3f}rad/s, 抑制因子:{:.2f}, 死区因子:{:.2f}",
      //          is_alignment_phase ? "对齐" : "跟随", current_max_angular_vel, current_suppression_factor,
      //          deadzone_factor);
    }
  }
  else
  {
    // 非地形自适应模式：使用平稳地形参数
    current_suppression_factor =
        is_alignment_phase ? alignment_params_.suppression_factor : smooth_terrain_params_.suppression_factor;

    // 应用平稳地形的抑制因子和死区因子
    smoothed_angular_vel *= current_suppression_factor * deadzone_factor;

    if (debug_)
    {
      // LOG_INFO("平稳地形控制 - 阶段:{}, 角速度限制:{:.3f}rad/s, 抑制因子:{:.2f}, 死区因子:{:.2f}",
      //          is_alignment_phase ? "对齐" : "跟随", current_max_angular_vel, current_suppression_factor,
      //          deadzone_factor);
    }
  }

  // 记录实际输出的角速度作为下次计算的参考
  prev_angular_velocity_ = smoothed_angular_vel;
  return smoothed_angular_vel;
}

double LineFollowController::calculateRotationVelocity(const double& angle_diff)
{
  // 使用sigmoid函数计算旋转因子
  double factor = 1.0 / (1.0 + std::exp(-rotation_angular_factor_ * std::abs(angle_diff)));

  // 在接近目标时使用余弦曲线平滑减速
  if (std::abs(angle_diff) < rotation_angle_threshold_)
  {
    double cosine_factor = rotation_angle_smooth_factor_ * (1.0 - std::cos(M_PI * std::abs(angle_diff) / rotation_angle_threshold_));
    factor *= cosine_factor;
  }

  // 计算旋转速度
  double rot_vel = max_rotation_angular_vel_ * factor;
  rot_vel = std::max(rot_vel, min_rotation_angular_vel_);

  // 根据角度差的符号决定旋转方向
  return (angle_diff > 0.0) ? rot_vel : -rot_vel;
}

double LineFollowController::computeCrossTrackError(double robot_x, double robot_y)
{
  if (global_plan_.poses.size() < 2)
  {
    return 0.0;
  }

  // 获取最近的路径段
  size_t nearest_segment_index = findNearestSegment(robot_x, robot_y);

  // 获取路径段的起点和终点
  const auto& pose1 = global_plan_.poses[nearest_segment_index].pose.position;
  const auto& pose2 = global_plan_.poses[nearest_segment_index + 1].pose.position;

  // 计算路径段向量和法向量
  double dx = pose2.x - pose1.x;
  double dy = pose2.y - pose1.y;
  double length = std::sqrt(dx * dx + dy * dy);

  if (length == 0.0)
  {
    return 0.0;
  }

  double nx = -dy / length;
  double ny = dx / length;

  // 计算机器人到路径段起点的向量
  double rx = robot_x - pose1.x;
  double ry = robot_y - pose1.y;

  // 计算横向偏差
  return rx * nx + ry * ny;
}

size_t LineFollowController::findNearestSegment(double robot_x, double robot_y)
{
  size_t nearest_index = 0;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i)
  {
    const auto& pose1 = global_plan_.poses[i].pose.position;
    const auto& pose2 = global_plan_.poses[i + 1].pose.position;

    double distance = distanceToSegment(robot_x, robot_y, pose1.x, pose1.y, pose2.x, pose2.y);

    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_index = i;
    }
  }

  return nearest_index;
}

double LineFollowController::distanceToSegment(double x, double y, double x1, double y1, double x2, double y2)
{
  double A = x - x1;
  double B = y - y1;
  double C = x2 - x1;
  double D = y2 - y1;

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = (len_sq != 0.0) ? (dot / len_sq) : -1.0;

  double xx, yy;
  if (param < 0.0)
  {
    xx = x1;
    yy = y1;
  }
  else if (param > 1.0)
  {
    xx = x2;
    yy = y2;
  }
  else
  {
    xx = x1 + param * C;
    yy = y1 + param * D;
  }

  double dx = x - xx;
  double dy = y - yy;
  return std::sqrt(dx * dx + dy * dy);
}

bool LineFollowController::handleStateAlignment(double robot_yaw,
                                                const geometry_msgs::msg::Quaternion& target_orientation,
                                                geometry_msgs::msg::TwistStamped& cmd_vel, bool is_backward)
{
  double target_yaw = tf2::getYaw(target_orientation);

  if (is_backward)
  {
    target_yaw += M_PI;
    target_yaw = std::atan2(std::sin(target_yaw), std::cos(target_yaw));
  }

  double yaw_error = angles::shortest_angular_distance(robot_yaw, target_yaw);

  // 检查是否已对齐
  if (isAlignedWithTarget(robot_yaw, target_orientation, is_backward))
  {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    last_yaw_error_ = 0.0;

    // 设置等待状态
    waiting_ = true;
    wait_start_time_ = std::chrono::steady_clock::now();

    return true;  // 对齐完成
  }

  // 计算角速度输出
  double angular_output = calculateRotationVelocity(yaw_error);

  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = angular_output;

  return false;  // 对齐未完成
}

void LineFollowController::handlePathFollowing(double robot_x, double robot_y,
                                               geometry_msgs::msg::TwistStamped& cmd_vel)
{
  // 计算距离
  double dx = target_pose_.pose.position.x - robot_x;
  double dy = target_pose_.pose.position.y - robot_y;
  double distance_to_target = std::sqrt(dx * dx + dy * dy);

  // 计算到原始目标点的距离（用于减速控制）
  double original_dx = original_target_pose_.pose.position.x - robot_x;
  double original_dy = original_target_pose_.pose.position.y - robot_y;
  double distance_to_original_target = std::sqrt(original_dx * original_dx + original_dy * original_dy);

  double start_dx = start_pose_.pose.position.x - robot_x;
  double start_dy = start_pose_.pose.position.y - robot_y;
  double distance_to_start = std::sqrt(start_dx * start_dx + start_dy * start_dy);

  // 获取下一个目标航点
  geometry_msgs::msg::PoseStamped target_waypoint = getNextWaypoint(robot_x, robot_y);

  // 计算路径方向
  double path_dx, path_dy;
  if (back_follow_)
  {
    path_dx = robot_x - target_waypoint.pose.position.x;
    path_dy = robot_y - target_waypoint.pose.position.y;
  }
  else
  {
    path_dx = target_waypoint.pose.position.x - robot_x;
    path_dy = target_waypoint.pose.position.y - robot_y;
  }

  // 计算路径方向航向
  double path_direction_yaw = std::atan2(path_dy, path_dx);
  double final_target_yaw = path_direction_yaw;  // 默认使用路径方向

  // 混合航向控制
  if (distance_to_start > m_alignment_distance_)
  {
    // 获取路径起始的理想航向
    double path_ideal_yaw = tf2::getYaw(start_pose_.pose.orientation);
    if (back_follow_)
    {
      path_ideal_yaw += M_PI;
      path_ideal_yaw = std::atan2(std::sin(path_ideal_yaw), std::cos(path_ideal_yaw));
    }

    // 使用地形自适应的航向权重进行混合控制
    double current_heading_weight = adaptive_params_.current_heading_weight;  // 路径理想航向权重
    double target_heading_weight = adaptive_params_.target_heading_weight;    // 指向目标点方向权重

    // 混合航向控制：理想航向 + 指向目标点的方向
    // path_ideal_yaw: 路径理想航向，保持直线性
    // path_direction_yaw: 指向目标点方向，包含横向偏差修正
    final_target_yaw = path_ideal_yaw * current_heading_weight + path_direction_yaw * target_heading_weight;

    if (robot_x == 0.0 || robot_y == 0.0)
    {
      final_target_yaw = path_ideal_yaw;
    }

    // 重置PID控制器（仅在第一次进入此阶段时）
    if (current_state_ == ControlState::ALIGNING_START)
    {
      pid_heading_controller_->reset();
      current_state_ = ControlState::FOLLOWING_PATH;
    }
  }
  else
  {
    // 在对齐阶段，直接使用路径方向作为目标航向
    final_target_yaw = path_direction_yaw;
  }

  // 计算线速度（使用到原始目标点的距离进行减速控制）
  double linear_speed = computeLinearSpeed(distance_to_original_target, distance_to_start);

  // 更新当前位姿（用于横向误差计算）
  current_pose_.pose.position.x = robot_x;
  current_pose_.pose.position.y = robot_y;

  // 计算航向误差和角速度
  double yaw_error = angles::shortest_angular_distance(robot_yaw_, final_target_yaw);
  double dt = 1.0 / 18.0;  // 固定时间步长
  double angular_output =
      computeAngularVelocity(yaw_error, dt, distance_to_original_target, distance_to_start, linear_speed);

  // 更新当前角速度
  current_angular_speed_ = angular_output;

  // 接近原始目标时停止转向
  if (distance_to_original_target < 0.07)
  {
    angular_output = 0.0;
  }

  // 设置速度命令
  cmd_vel.twist.linear.x = linear_speed;
  cmd_vel.twist.angular.z = angular_output;

  // 后退模式时取反线速度
  if (back_follow_)
  {
    cmd_vel.twist.linear.x = -cmd_vel.twist.linear.x;
  }

  // 计算横向偏差用于监控
  double cross_track_error = computeCrossTrackError(robot_x, robot_y);

  // 记录路径跟随数据 - 无论IMU地形自适应是否启用都判断死区状态
  bool in_deadzone = false;
  
  // 调试输出：检查当前状态
  if (debug_)
  {
    std::string state_name;
    switch (current_state_) {
      case ControlState::IDLE: state_name = "IDLE"; break;
      case ControlState::ALIGNING_START: state_name = "ALIGNING_START"; break;
      case ControlState::FOLLOWING_PATH: state_name = "FOLLOWING_PATH"; break;
      case ControlState::ALIGNING_END: state_name = "ALIGNING_END"; break;
      case ControlState::GOAL_REACHED: state_name = "GOAL_REACHED"; break;
      case ControlState::WAITING: state_name = "WAITING"; break;
      default: state_name = "UNKNOWN"; break;
    }
    LOG_INFO("当前控制状态: {}, IMU地形自适应: {}", state_name, enable_imu_terrain_adaptation_ ? "启用" : "禁用");
  }
  
  // 临时修改：无论什么状态都判断死区（用于调试）
  // if (current_state_ == ControlState::FOLLOWING_PATH)
  if (true)  // 临时测试用
  {
    if (enable_imu_terrain_adaptation_)
    {
      // 使用自适应地形参数判断死区
      in_deadzone = (std::abs(cross_track_error) < adaptive_params_.current_cross_track_deadzone &&
                     std::abs(yaw_error) < adaptive_params_.current_yaw_deadzone);
    }
    else
    {
      // 使用平稳地形参数判断死区
      in_deadzone = (std::abs(cross_track_error) < smooth_terrain_params_.cross_track_deadzone &&
                     std::abs(yaw_error) < smooth_terrain_params_.yaw_deadzone);
      
      // 调试输出：检查平稳地形死区判断
      if (debug_)
      {
        // LOG_INFO("平稳地形死区检查 - 横向误差:{:.6f} < {:.6f}? {}, 航向误差:{:.6f} < {:.6f}? {}, 死区状态:{}",
        //          std::abs(cross_track_error), smooth_terrain_params_.cross_track_deadzone, 
        //          std::abs(cross_track_error) < smooth_terrain_params_.cross_track_deadzone,
        //          std::abs(yaw_error), smooth_terrain_params_.yaw_deadzone,
        //          std::abs(yaw_error) < smooth_terrain_params_.yaw_deadzone,
        //          in_deadzone ? "是" : "否");
      }
    }
  }


  // 发布控制日志数据 - 根据IMU地形自适应状态选择参数
  double log_cross_track_deadzone, log_yaw_deadzone, log_max_angular_vel, log_max_angular_accel, log_suppression_factor;
  
  if (enable_imu_terrain_adaptation_) {
    // 使用自适应地形参数
    log_cross_track_deadzone = adaptive_params_.current_cross_track_deadzone;
    log_yaw_deadzone = adaptive_params_.current_yaw_deadzone;
    log_max_angular_vel = adaptive_params_.current_max_angular_vel;
    log_max_angular_accel = adaptive_params_.current_max_angular_accel;
    log_suppression_factor = adaptive_params_.current_suppression_factor;
  } else {
    // 使用平稳地形参数
    log_cross_track_deadzone = smooth_terrain_params_.cross_track_deadzone;
    log_yaw_deadzone = smooth_terrain_params_.yaw_deadzone;
    log_max_angular_vel = smooth_terrain_params_.max_angular_vel;
    log_max_angular_accel = smooth_terrain_params_.max_angular_accel;
    log_suppression_factor = smooth_terrain_params_.suppression_factor;
  }
  
  publishControlLogData(yaw_error, cross_track_error, cmd_vel, back_follow_, in_deadzone, 
                       control_data_.deadzone_factor, control_data_.d_adaptive_factor,
                       log_max_angular_vel, log_suppression_factor,
                       log_cross_track_deadzone, log_yaw_deadzone,
                       log_max_angular_accel, control_data_.angular_acceleration,
                       enable_imu_terrain_adaptation_);

  if (debug_)
  {
    // LOG_INFO("路径跟随 - 航向误差: {:.4f}, 横向误差: {:.4f}, 速度: [{:.3f}, {:.3f}], 后退: {}", yaw_error,
    //          cross_track_error, cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, back_follow_);

    // if (enable_imu_terrain_adaptation_ && current_state_ == ControlState::FOLLOWING_PATH)
    // {
    //   LOG_INFO("地形自适应控制 - 死区状态: {}, 角速度限制: {:.3f}, 抑制因子: {:.2f}", in_deadzone ? "是" : "否",
    //            adaptive_params_.current_max_angular_vel, adaptive_params_.current_suppression_factor);
    // }
    // LOG_INFO(" ");
  }
}

bool LineFollowController::handleWaitingState(geometry_msgs::msg::TwistStamped& cmd_vel)
{
  auto current_time = std::chrono::steady_clock::now();
  double wait_time = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - wait_start_time_).count();

  if (wait_time >= wait_duration_)
  {
    waiting_ = false;
    return false;  // 等待结束
  }

  // 等待期间保持静止
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  return true;  // 继续等待
}

double LineFollowController::getDeltaTime()
{
  auto current_time = std::chrono::steady_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - last_time_).count();
  last_time_ = current_time;

  // 防止异常时间间隔
  if (dt <= 0.0 || std::isnan(dt) || std::isinf(dt) || dt > 1.0)
  {
    dt = 0.025;  // 默认值
  }

  return dt;
}

void LineFollowController::exportDebugData(const std::string& file_path, const std::vector<std::vector<double>>& data)
{
  std::lock_guard<std::mutex> lock(file_mutex_);

  // 获取文件路径的目录部分
  std::filesystem::path file_path_obj(file_path);
  std::filesystem::path directory = file_path_obj.parent_path();
  
  // 如果目录不存在，按链路创建目录
  if (!directory.empty() && !std::filesystem::exists(directory))
  {
    try
    {
      std::filesystem::create_directories(directory);
      LOG_INFO("创建目录: {}", directory.string());
    }
    catch (const std::exception& e)
    {
      LOG_ERROR("无法创建目录 {}: {}", directory.string(), e.what());
      return;
    }
  }

  std::ofstream file(file_path);
  if (!file.is_open())
  {
    LOG_ERROR("无法打开文件进行数据导出: {}", file_path);
    return;
  }

  file << std::fixed << std::setprecision(10);
  file << "x,y\n";

  for (const auto& row : data)
  {
    for (size_t i = 0; i < row.size(); ++i)
    {
      file << row[i];
      if (i < row.size() - 1)
        file << ",";
    }
    file << "\n";
  }

  file.close();
  // LOG_INFO("数据导出成功: {} 行数据到 {}", data.size(), file_path);
}

bool LineFollowController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                   const geometry_msgs::msg::Twist& velocity,
                                                   geometry_msgs::msg::TwistStamped& cmd_vel)
{
  // 标记未使用的参数以抑制编译警告
  (void)velocity;

  // 注意：IMU数据处理已移至独立线程，此处无需调用spinImuNode()

  // 检查是否有路径
  if (!received_plan_)
  {
    LOG_WARN("尚未接收到路径");
    return false;
  }

  // 检查是否已到达目标
  if (goal_reached_)
  {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return true;
  }

  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0)
  {
    initializeFilters();
  }

  // 处理重置请求
  if (reset_required_)
  {
    // 重新计算起点（用于动态路径调整）
    double robot_x = pose.pose.position.x;
    double robot_y = pose.pose.position.y;

    if (!short_path_)
    {
      start_pose_ = getNextWaypoint(robot_x, robot_y);
      double dx = start_pose_.pose.position.x - target_pose_.pose.position.x;
      double dy = start_pose_.pose.position.y - target_pose_.pose.position.y;
      path_length_ = std::sqrt(dx * dx + dy * dy);
    }

    reset_required_ = false;
  }

  // 处理等待状态
  if (waiting_)
  {
    if (handleWaitingState(cmd_vel))
    {
      return true;  // 继续等待
    }
  }

  // 获取机器人当前姿态
  robot_yaw_ = tf2::getYaw(pose.pose.orientation);
  double robot_x = pose.pose.position.x;
  double robot_y = pose.pose.position.y;

  // 位置滤波处理
  double filtered_x_ = h_x_filter_.filter(robot_x);
  double filtered_y_ = h_y_filter_.filter(robot_y);

  double smoothed_robot_x = filtered_x_;
  double smoothed_robot_y = filtered_y_;

  smoothed_robot_x = x_filter_.filter(smoothed_robot_x);
  smoothed_robot_y = y_filter_.filter(smoothed_robot_y);

  // ================================
  // 定位数据处理
  // 默认使用滤波后位置，仅在FOLLOWING_PATH阶段使用虚拟位置
  // ================================
  double control_x = smoothed_robot_x;  // 默认使用滤波后位置
  double control_y = smoothed_robot_y;

  // 记录调试数据
  if (debug_)
  {
    original_path_.push_back({ filtered_x_, filtered_y_ });
    filtered_path_.push_back({ smoothed_robot_x, smoothed_robot_y });
  }

  // 状态机处理
  switch (current_state_)
  {
    case ControlState::ALIGNING_START: {
      // 对齐起始航向
      if (handleStateAlignment(robot_yaw_, start_pose_.pose.orientation, cmd_vel, back_follow_))
      {
        // ================================
        // 对齐完成，即将进入路径跟随阶段
        // 此时全站仪数据可靠，直接赋值进行强制校准
        // ================================
        if (virtual_tracking_config_.enabled)
        {
          // 使用可靠的滤波位置数据直接设置虚拟位置（强制校准）
          if (!virtual_tracker_.isInitialized()) {
            virtual_tracker_.initialize(control_x, control_y, robot_yaw_);
          } else {
            // 直接赋值可靠位置（因为对齐阶段后数据可靠）
            virtual_tracker_.initialize(control_x, control_y, robot_yaw_);
          }
        }
        
        current_state_ = ControlState::FOLLOWING_PATH;
      }
      break;
    }

    case ControlState::FOLLOWING_PATH: {
      // ================================
      // 仅在路径跟随阶段使用虚拟位置跟踪（超高精度5mm）
      // ================================
      if (virtual_tracking_config_.enabled && virtual_tracker_.isInitialized())
      {
        // 获取当前地形类型
        TerrainType current_terrain = TerrainType::SMOOTH;
        if (enable_imu_terrain_adaptation_) {
          current_terrain = terrain_data_.current_type;
        }
        
        // 使用全站仪滤波数据修正虚拟位置
        virtual_tracker_.correctWithMeasurement(smoothed_robot_x, smoothed_robot_y, current_terrain);
        
        // 使用虚拟位置进行超高精度控制
        control_x = virtual_tracker_.getVirtualX();
        control_y = virtual_tracker_.getVirtualY();
      }
      
      // 检查是否到达终点（使用虚拟控制位置）
      if (isBeyondGoal(control_x, control_y))
      {
        current_state_ = ControlState::ALIGNING_END;
        current_linear_speed_ = 0.0;
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        waiting_ = true;
        wait_start_time_ = std::chrono::steady_clock::now();
        return true;
      }

      // 路径跟随（使用虚拟控制位置实现超高精度）
      handlePathFollowing(control_x, control_y, cmd_vel);
      break;
    }

    case ControlState::ALIGNING_END: {
      // 对齐原始终点航向
      if (handleStateAlignment(robot_yaw_, original_target_pose_.pose.orientation, cmd_vel, false))
      {
        current_state_ = ControlState::GOAL_REACHED;
        goal_reached_ = true;


        // 导出IMU地形数据
        if (enable_imu_terrain_logging_)
        {
          exportImuTerrainData();
        }

        // 导出调试数据
        if (debug_)
        {
          // 移除起始和终点的重复数据
          if (original_path_.size() > 2)
          {
            original_path_.erase(original_path_.begin() + 2, original_path_.begin() + 4);
            filtered_path_.erase(filtered_path_.begin() + 2, filtered_path_.begin() + 4);
          }

          std::string timestamp = std::to_string(
              std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch())
                  .count());

          // exportDebugData("/home/daosn_robotics/zyq_ws/path_l/_" + timestamp + "_line.csv", original_path_);
          exportDebugData("/home/daosn_robotics/path_sg/_" + timestamp + "_line.csv", filtered_path_);
        }
      }
      break;
    }

    case ControlState::GOAL_REACHED: {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      break;
    }

    default: {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      break;
    }
  }

  // ================================
  // 更新虚拟位置跟踪器（仅在FOLLOWING_PATH阶段）
  // ================================
  if (virtual_tracking_config_.enabled && virtual_tracker_.isInitialized() && 
      current_state_ == ControlState::FOLLOWING_PATH)
  {
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - last_time_).count();
    dt = std::clamp(dt, 0.001, 0.1);  // 限制dt在合理范围内
    
    // 使用输出的控制命令更新虚拟位置
    virtual_tracker_.updateVirtualPose(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, dt);
  }

  return true;
}

bool LineFollowController::isGoalReached()
{
  return goal_reached_;
}

bool LineFollowController::cancel()
{

  // 导出IMU地形数据
  if (!imu_terrain_data_.empty())
  {
    exportImuTerrainData();
  }

  resetControllerState();
  // LOG_INFO("控制器已取消");
  return true;
}

void LineFollowController::extendPath()
{
  // 如果路径太短,不进行延长
  if (original_path_length_ < 0.01)
  {
    return;
  }

  // 计算路径方向向量
  double dx = target_pose_.pose.position.x - start_pose_.pose.position.x;
  double dy = target_pose_.pose.position.y - start_pose_.pose.position.y;

  // 归一化方向向量
  double norm = std::sqrt(dx * dx + dy * dy);
  double unit_dx = dx / norm;
  double unit_dy = dy / norm;

  // 计算需要添加的路径点数量
  int num_points = static_cast<int>(PATH_EXTENSION_LENGTH / PATH_POINT_INTERVAL);

  // 添加延长的路径点
  for (int i = 1; i <= num_points; ++i)
  {
    geometry_msgs::msg::PoseStamped new_pose;
    new_pose.header = target_pose_.header;

    // 计算新路径点位置
    double extension_distance = i * PATH_POINT_INTERVAL;
    new_pose.pose.position.x = target_pose_.pose.position.x + unit_dx * extension_distance;
    new_pose.pose.position.y = target_pose_.pose.position.y + unit_dy * extension_distance;
    new_pose.pose.position.z = target_pose_.pose.position.z;

    // 保持相同的朝向
    new_pose.pose.orientation = target_pose_.pose.orientation;

    // 添加到路径中
    global_plan_.poses.push_back(new_pose);
  }

  // 更新目标位姿为延长后的最后一个点
  target_pose_ = global_plan_.poses.back();
  end_pose_ = target_pose_;

  LOG_INFO("路径已延长 {:.2f}m,添加了 {} 个路径点", PATH_EXTENSION_LENGTH, num_points);
}

std::string LineFollowController::generateTaskId()
{
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
  return ss.str();
}



/**
 * @brief 初始化IMU订阅,使用独立线程处理IMU节点
 *
 * 设计理念：
 * - 职责分离：创建专门的IMU内部节点,只负责IMU数据订阅
 * - 独立线程：使用200Hz频率的独立线程处理IMU数据,确保实时性
 * - 模块化：IMU功能与主控制逻辑解耦
 */
void LineFollowController::initializeImuSubscription()
{
  if (!enable_imu_terrain_adaptation_)
  {
    LOG_INFO("IMU地形自适应控制未启用,跳过IMU订阅");
    return;
  }

  // 创建专门用于IMU订阅的内部节点（独立于外部节点）
  if (!imu_node_)
  {
    rclcpp::NodeOptions imu_node_options;
    imu_node_options.use_intra_process_comms(true);  // 启用进程内通信优化

    imu_node_ = rclcpp::Node::make_shared("line_follow_imu_subscriber", imu_node_options);
    LOG_INFO("创建IMU专用内部节点: line_follow_imu_subscriber");
  }

  // 使用内部IMU节点创建订阅
  imu_subscription_ = imu_node_->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(), std::bind(&LineFollowController::imuCallback, this, std::placeholders::_1));

  LOG_INFO("IMU订阅已初始化: /imu (订阅创建完成,线程将延迟启动)");
}

/**
 * @brief 启动IMU处理线程
 *
 * 在对象完全构造后安全启动IMU处理线程
 */
void LineFollowController::startImuThread()
{
  if (!enable_imu_terrain_adaptation_ || !imu_node_ || imu_thread_running_.load())
  {
    return;  // 未启用IMU功能、节点未创建或线程已运行
  }

  // 启动独立线程处理IMU数据，200Hz频率
  imu_thread_running_ = true;
  imu_thread_ = std::thread(&LineFollowController::imuThreadLoop, this);

  LOG_INFO("IMU处理线程已启动 (200Hz频率)");
}

/**
 * @brief 处理IMU专用节点的消息 (已弃用)
 *
 * 注意：此方法已被独立线程取代，保留仅为兼容性
 */
void LineFollowController::spinImuNode()
{
  // 已弃用，IMU数据处理已移至独立线程
}

/**
 * @brief IMU独立线程循环函数
 *
 * 在200Hz频率下处理IMU节点的消息，确保实时性
 */
void LineFollowController::imuThreadLoop()
{
  const std::chrono::microseconds loop_period(5000);  // 200Hz = 5ms period

  while (imu_thread_running_.load())
  {
    static int count = 0;
    if (++count >= 400)
    {
      updateParameters();
      count = 0;
    }

    auto start_time = std::chrono::steady_clock::now();

    if (imu_node_ && enable_imu_terrain_adaptation_)
    {
      // 非阻塞式处理IMU节点的消息
      rclcpp::spin_some(imu_node_);
    }

    // 计算剩余时间并休眠
    auto end_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    if (elapsed < loop_period)
    {
      std::this_thread::sleep_for(loop_period - elapsed);
    }
  }
}

void LineFollowController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // 从四元数中提取姿态角（注意：yaw角与机器人航向无关，仅用于地形检测）
  double roll, pitch, yaw;  // yaw角不用于导航，仅作为地形分析的辅助数据
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // 更新当前翻滚角（用于调试和记录）
  current_roll_angle_ = roll;

  // 100%信任IMU数据，无滤波
  filtered_roll_angle_ = current_roll_angle_;

  // 每150次回调输出一次IMU数据状态，用于监控数据质量
  static int callback_count = 0;
  if (++callback_count >= 150)
  {
    // LOG_INFO("");
    // LOG_INFO("IMU数据监控 - Roll:{:.6f}, Pitch:{:.6f}, Yaw:{:.6f}, 历史数据量:{}", roll, pitch, yaw,
    //          imu_history_.size());
    callback_count = 0;
  }

  // 地形自适应控制：更新IMU历史数据
  updateImuHistory(msg);

  // 1. 提取地形特征
  TerrainFeatures features = extractTerrainFeatures();

  // 1.5. 自适应基础地形管理已移除（立即切换模式）

  // 2. 立即地形分类（无过渡控制）
  TerrainType current_terrain = classifyTerrainWithPriority(features);

  // 3. 计算分类置信度（基于特征强度）
  double confidence = 1.0;  // 基础置信度
  if (current_terrain == TerrainType::SUDDEN_CHANGE)
  {
    confidence = std::min(1.0, features.max_gradient / sudden_change_gradient_threshold_);
  }
  else if (current_terrain == TerrainType::GENTLE_UNDULATION)
  {
    confidence = std::min(1.0, features.low_freq_energy_ratio / gentle_low_freq_energy_threshold_);
  }
  else if (current_terrain == TerrainType::FREQUENT_BUMPS)
  {
    confidence = std::min(1.0, features.zero_crossing_rate / frequent_zero_crossing_threshold_);
  }

  // 4. 更新地形分析数据（线程安全）
  {
    std::lock_guard<std::mutex> lock(terrain_data_mutex_);
    terrain_data_.current_type = current_terrain;
    terrain_data_.features = features;
    terrain_data_.confidence_score = confidence;
    terrain_data_.last_update = std::chrono::steady_clock::now();
  }

  // 5. 应用控制参数自适应
  adaptControlParameters(current_terrain, confidence);

  // 记录IMU地形数据（按配置的频率）
  if (enable_imu_terrain_logging_)
  {
    auto current_time = std::chrono::steady_clock::now();
    double time_since_last_log =
        std::chrono::duration_cast<std::chrono::duration<double>>(current_time - last_log_time_).count();

    if (time_since_last_log >= (1.0 / logging_frequency_))
    {
      recordImuTerrainData();
      last_log_time_ = current_time;
    }
  }
}

void LineFollowController::updateImuHistory(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // 从四元数中提取翻滚角
  double roll, pitch, yaw;
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // 创建新的IMU数据点（存储翻滚角和俯仰角用于突变检测）
  ImuDataPoint data_point;
  data_point.roll = roll;
  data_point.pitch = pitch;  // 添加俯仰角用于突变检测
  data_point.timestamp = std::chrono::steady_clock::now();

  // 添加到历史队列
  imu_history_.push_back(data_point);

  // 维护队列大小,移除过旧的数据
  while (imu_history_.size() > max_imu_history_size_)
  {
    imu_history_.pop_front();
  }

  // 清理超出时间窗口的数据
  auto current_time = std::chrono::steady_clock::now();
  auto cutoff_time = current_time - std::chrono::duration<double>(imu_window_size_);

  while (!imu_history_.empty() && imu_history_.front().timestamp < cutoff_time)
  {
    imu_history_.pop_front();
  }
}

/**
 * @brief 新算法：提取地形特征（基于航空级IMU精度）
 * @return 提取的地形特征数据
 */
LineFollowController::TerrainFeatures LineFollowController::extractTerrainFeatures()
{
  TerrainFeatures features;

  if (imu_history_.size() < 40)
  {                                 // 基于200Hz,至少需要0.2秒数据
    return terrain_data_.features;  // 返回上次的特征
  }

  // 特征1：计算最大梯度（突变检测 - 最高优先级）
  features.max_gradient = computeMaxGradient();

  // 特征2：计算过零率（频繁颠簸检测）
  features.zero_crossing_rate = computeZeroCrossingRate();

  // 特征3：计算低频能量占比（平缓凹凸检测）
  features.low_freq_energy_ratio = computeLowFreqEnergyRatio();

  features.last_update = std::chrono::steady_clock::now();

  if (debug_)
  {
    // 地形特征数据已记录到文件，无需控制台输出
  }

  return features;
}

/**
 * @brief 阶段1实现：突变检测算法（增强版：同时使用翻滚角和俯仰角）
 * @return 最大梯度值（rad/sample）
 */
double LineFollowController::computeMaxGradient()
{
  if (imu_history_.size() < 2)
  {
    return 0.0;
  }

  double max_gradient = 0.0;
  auto prev_it = imu_history_.begin();
  auto curr_it = std::next(prev_it);

  // 计算相邻样本间的最大梯度（同时考虑翻滚角和俯仰角）
  while (curr_it != imu_history_.end())
  {
    // 计算翻滚角梯度（侧向地形变化）
    double roll_gradient = std::abs(curr_it->roll - prev_it->roll);

    // 计算俯仰角梯度（前向地形变化 - 对突起、坑洍更敏感）
    double pitch_gradient = std::abs(curr_it->pitch - prev_it->pitch);

    // 使用加权组合：俯仰角权重更高，因为对前向突变更敏感
    double combined_gradient = 0.6 * pitch_gradient + 0.4 * roll_gradient;

    max_gradient = std::max(max_gradient, combined_gradient);

    prev_it = curr_it;
    ++curr_it;
  }

  if (debug_ && max_gradient > sudden_change_gradient_threshold_)
  {
    // 突变检测事件已记录到文件，无需控制台输出
  }

  return max_gradient;
}

/**
 * @brief 阶段2实现：过零率计算（频繁颠簸检测）- 改进版，增加噪声阈值
 * @return 过零率（次数/秒）
 */
double LineFollowController::computeZeroCrossingRate()
{
  if (imu_history_.size() < 10)
  {
    return 0.0;
  }

  // 提取翻滚角和俯仰角序列
  std::vector<double> roll_data;
  std::vector<double> pitch_data;
  roll_data.reserve(imu_history_.size());
  pitch_data.reserve(imu_history_.size());
  
  for (const auto& point : imu_history_)
  {
    roll_data.push_back(point.roll);
    pitch_data.push_back(point.pitch);
  }

  // 计算去均值序列
  auto computeZeroCrossingsWithThreshold = [](const std::vector<double>& data, double noise_threshold) -> int {
    if (data.size() < 3) return 0;
    
    // 计算均值
    double mean = 0.0;
    for (double val : data)
    {
      mean += val;
    }
    mean /= data.size();
    
    // 计算标准差来自适应确定噪声阈值
    double variance = 0.0;
    for (double val : data)
    {
      double diff = val - mean;
      variance += diff * diff;
    }
    variance /= data.size();
    double std_dev = std::sqrt(variance);
    
    // 如果标准差太小，说明信号很稳定，设置较高的阈值
    double adaptive_threshold = std::max(noise_threshold, std_dev * 0.5);
    
    // 统计有效过零次数（只有变化幅度超过阈值才算）
    int zero_crossings = 0;
    double prev_value = data[0] - mean;
    
    for (size_t i = 1; i < data.size(); ++i)
    {
      double curr_value = data[i] - mean;
      
      // 只有当前值和前一个值的绝对值都超过阈值时，才考虑过零
      if (std::abs(prev_value) > adaptive_threshold && std::abs(curr_value) > adaptive_threshold)
      {
        if ((prev_value > 0 && curr_value < 0) || (prev_value < 0 && curr_value > 0))
        {
          zero_crossings++;
        }
      }
      prev_value = curr_value;
    }
    
    return zero_crossings;
  };

  // 设置噪声阈值（弧度），对应约0.1度的变化
  const double noise_threshold = 0.0017;  // 约0.1度
  
  // 分别计算翻滚角和俯仰角的过零次数
  int roll_crossings = computeZeroCrossingsWithThreshold(roll_data, noise_threshold);
  int pitch_crossings = computeZeroCrossingsWithThreshold(pitch_data, noise_threshold);
  
  // 使用较高的过零次数（因为颠簸地形通常在多个轴上都有表现）
  int total_crossings = std::max(roll_crossings, pitch_crossings);
  
  // 转换为每秒的过零率
  double duration = std::chrono::duration_cast<std::chrono::duration<double>>(
      imu_history_.back().timestamp - imu_history_.front().timestamp).count();
  
  if (duration <= 0.1)  // 防止除零，且时间窗口太小时不可信
  {
    return 0.0;
  }
  
  double zero_crossing_rate = static_cast<double>(total_crossings) / duration;
  
  // 对于频繁颠簸检测，过零率应该相对较高且稳定
  // 如果计算出的过零率过低，可能是噪声，返回更保守的值
  if (zero_crossing_rate < 0.5)
  {
    zero_crossing_rate = 0.0;  // 认为是静态噪声
  }
  
  return zero_crossing_rate;
}

/**
 * @brief 阶段3实现：低频能量占比计算（平缓凹凸检测）
 * @return 低频能量占总能量的比例 (0-1)
 */
double LineFollowController::computeLowFreqEnergyRatio()
{
  if (imu_history_.size() < 20)
  {
    return 0.0;
  }

  // 提取翻滚角和俯仰角数据序列
  std::vector<double> roll_data;
  std::vector<double> pitch_data;
  roll_data.reserve(imu_history_.size());
  pitch_data.reserve(imu_history_.size());
  
  for (const auto& point : imu_history_)
  {
    roll_data.push_back(point.roll);
    pitch_data.push_back(point.pitch);
  }

  // 去均值化
  auto detrend = [](std::vector<double>& data) {
    double mean = 0.0;
    for (double val : data)
    {
      mean += val;
    }
    mean /= data.size();
    
    for (double& val : data)
    {
      val -= mean;
    }
  };

  detrend(roll_data);
  detrend(pitch_data);

  // 计算变化率序列（一阶差分）来突出动态变化
  auto computeChanges = [](const std::vector<double>& data) -> std::vector<double> {
    std::vector<double> changes;
    if (data.size() < 2) return changes;
    
    changes.reserve(data.size() - 1);
    for (size_t i = 1; i < data.size(); ++i)
    {
      changes.push_back(data[i] - data[i-1]);
    }
    return changes;
  };

  std::vector<double> roll_changes = computeChanges(roll_data);
  std::vector<double> pitch_changes = computeChanges(pitch_data);
  
  if (roll_changes.empty() || pitch_changes.empty())
  {
    return 0.0;
  }

  // 计算变化率的标准差作为总变化强度
  auto computeStd = [](const std::vector<double>& data) -> double {
    if (data.empty()) return 0.0;
    
    double mean = 0.0;
    for (double val : data)
    {
      mean += val;
    }
    mean /= data.size();
    
    double variance = 0.0;
    for (double val : data)
    {
      double diff = val - mean;
      variance += diff * diff;
    }
    variance /= data.size();
    
    return std::sqrt(variance);
  };

  double roll_total_std = computeStd(roll_changes);
  double pitch_total_std = computeStd(pitch_changes);
  
  // 如果变化太小，认为是静止状态
  if (roll_total_std < 1e-6 && pitch_total_std < 1e-6)
  {
    return 0.0;
  }

  // 使用更长窗口的滑动平均来提取真正的低频趋势
  const int long_window = std::min(static_cast<int>(roll_changes.size() / 2), 50);
  
  auto extractLowFreqTrend = [long_window](const std::vector<double>& data) -> std::vector<double> {
    std::vector<double> trend;
    if (data.size() < static_cast<size_t>(long_window)) return trend;
    
    trend.reserve(data.size());
    
    for (size_t i = 0; i < data.size(); ++i)
    {
      double sum = 0.0;
      int count = 0;
      
      // 使用较大的窗口来平滑
      int start = std::max(0, static_cast<int>(i) - long_window / 2);
      int end = std::min(static_cast<int>(data.size() - 1), static_cast<int>(i) + long_window / 2);
      
      for (int j = start; j <= end; ++j)
      {
        sum += data[j];
        count++;
      }
      
      trend.push_back(sum / count);
    }
    
    return trend;
  };

  std::vector<double> roll_trend = extractLowFreqTrend(roll_changes);
  std::vector<double> pitch_trend = extractLowFreqTrend(pitch_changes);

  if (roll_trend.empty() || pitch_trend.empty())
  {
    return 0.0;
  }

  // 计算低频趋势的标准差
  double roll_trend_std = computeStd(roll_trend);
  double pitch_trend_std = computeStd(pitch_trend);

  // 计算高频成分（原始变化 - 低频趋势）的标准差
  auto computeHighFreqStd = [&computeStd](const std::vector<double>& original, 
                                          const std::vector<double>& trend) -> double {
    if (original.size() != trend.size()) return 0.0;
    
    std::vector<double> high_freq;
    high_freq.reserve(original.size());
    
    for (size_t i = 0; i < original.size(); ++i)
    {
      high_freq.push_back(original[i] - trend[i]);
    }
    
    return computeStd(high_freq);
  };

  double roll_high_std = computeHighFreqStd(roll_changes, roll_trend);
  double pitch_high_std = computeHighFreqStd(pitch_changes, pitch_trend);

  // 计算低频能量占比
  double roll_ratio = 0.0;
  double pitch_ratio = 0.0;
  
  double roll_total_energy = roll_trend_std * roll_trend_std + roll_high_std * roll_high_std;
  double pitch_total_energy = pitch_trend_std * pitch_trend_std + pitch_high_std * pitch_high_std;
  
  if (roll_total_energy > 1e-10)
  {
    roll_ratio = (roll_trend_std * roll_trend_std) / roll_total_energy;
  }
  
  if (pitch_total_energy > 1e-10)
  {
    pitch_ratio = (pitch_trend_std * pitch_trend_std) / pitch_total_energy;
  }

  // 加权组合：俯仰角权重更高
  double combined_ratio = 0.6 * pitch_ratio + 0.4 * roll_ratio;
  
  // 对于地形检测，我们关心的是持续的低频变化，而不是瞬时噪声
  // 如果总体变化太小，即使低频占比高也应该返回较小值
  double total_activity = std::max(roll_total_std, pitch_total_std);
  if (total_activity < 0.001)  // 活动强度阈值
  {
    combined_ratio *= 0.1;  // 大幅降低占比
  }
  
  return std::max(0.0, std::min(1.0, combined_ratio));
}

/**
 * @brief 核心算法：基于优先级的地形分类（突变 → 平缓凹凸 → 频繁颠簸）
 * @param features 提取的地形特征
 * @return 分类后的地形类型
 */
LineFollowController::TerrainType LineFollowController::classifyTerrainWithPriority(const TerrainFeatures& features)
{
  static int count = 0;
  static bool is_print = false;

  is_print = false;
  if (++count >= 150)
  {
    is_print = true;
    count = 0;
  }

  terrain_data_.angle_vel_dynamic_factor = 1.0;
  terrain_data_.line_vel_dynamic_factor = 1.0;

  if (debug_ && is_print)
  {
    // LOG_INFO("低频能量占比:{:.10f} > {:.10f},最大梯度:{:.10f} > {:.10f},过零率:{:.10f} > {:.10f}",
    //          features.low_freq_energy_ratio, gentle_low_freq_energy_threshold_, features.max_gradient,
    //          sudden_change_gradient_threshold_, features.zero_crossing_rate, frequent_zero_crossing_threshold_);
  }

  // 优先级1：突变检测（最高优先级 - 安全关键）
  if (features.max_gradient > sudden_change_gradient_threshold_)
  {
    // 降速
    terrain_data_.line_vel_dynamic_factor = 0.7;

    TerrainType result = TerrainType::SUDDEN_CHANGE;

    // 发布地形日志数据（仅在debug模式下）
    if (debug_ && is_print)
    {
      double confidence = std::min(1.0, features.max_gradient / sudden_change_gradient_threshold_);
      publishTerrainLogData(features, result, confidence);
      LOG_INFO("当前地形：突变地形");
    }
    // 地形分类结果已记录到文件
    return result;
  }

  // 优先级2：平缓凹凸检测（中优先级 - 影响跟踪精度）
  if (features.low_freq_energy_ratio > gentle_low_freq_energy_threshold_)
  {
    TerrainType result = TerrainType::GENTLE_UNDULATION;

    // 发布地形日志数据（仅在debug模式下）
    if (debug_ && is_print)
    {
      double confidence = std::min(1.0, features.low_freq_energy_ratio / gentle_low_freq_energy_threshold_);
      publishTerrainLogData(features, result, confidence);
      LOG_INFO("当前地形：平缓凹凸地形");
    }

    // 地形分类结果已记录到文件
    return result;
  }

  // 优先级3：频繁颠簸检测（低优先级 - 影响稳定性）
  if (features.zero_crossing_rate > frequent_zero_crossing_threshold_)
  {
    TerrainType result = TerrainType::FREQUENT_BUMPS;

    // 发布地形日志数据（仅在debug模式下）
    if (debug_ && is_print)
    {
      double confidence = std::min(1.0, features.zero_crossing_rate / frequent_zero_crossing_threshold_);
      publishTerrainLogData(features, result, confidence);
      LOG_INFO("当前地形：频繁颠簸地形");
    }

    return result;
  }

  TerrainType result = TerrainType::SMOOTH;
  if (debug_ && is_print)
  {
    LOG_INFO("当前地形：平稳地形");
  }

  // 发布地形日志数据（仅在debug模式下）
  if (debug_ && is_print)
  {
    publishTerrainLogData(features, result, 0.8);  // 平稳地形的默认置信度
  }

  return result;
}

/**
 * @brief 高级控制参数自适应算法
 * @param terrain_type 识别的地形类型
 * @param confidence 分类置信度（0-1）
 */
void LineFollowController::adaptControlParameters(TerrainType terrain_type, double confidence)
{
  // 立即切换模式：直接使用对应的地形参数
  TerrainControlParams terrain_params = selectTerrainParams(terrain_type);

  // 应用置信度加权的参数调整 - 只更新跟随阶段参数
  AdaptiveControlParams target_params = adaptive_params_;
  target_params.current_cross_track_deadzone = terrain_params.cross_track_deadzone;
  target_params.current_yaw_deadzone = terrain_params.yaw_deadzone;
  target_params.current_max_angular_vel = terrain_params.max_angular_vel;
  target_params.current_max_angular_accel = terrain_params.max_angular_accel;
  target_params.current_suppression_factor = terrain_params.suppression_factor;
  target_params.current_heading_weight = terrain_params.current_heading_weight;
  target_params.target_heading_weight = terrain_params.target_heading_weight;

  // 更新跟随阶段参数
  adaptive_params_.current_cross_track_deadzone = confidence * target_params.current_cross_track_deadzone +
                                                  (1.0 - confidence) * adaptive_params_.current_cross_track_deadzone;

  adaptive_params_.current_yaw_deadzone =
      confidence * target_params.current_yaw_deadzone + (1.0 - confidence) * adaptive_params_.current_yaw_deadzone;

  adaptive_params_.current_max_angular_vel = confidence * target_params.current_max_angular_vel +
                                             (1.0 - confidence) * adaptive_params_.current_max_angular_vel;

  adaptive_params_.current_max_angular_accel = confidence * target_params.current_max_angular_accel +
                                               (1.0 - confidence) * adaptive_params_.current_max_angular_accel;

  adaptive_params_.current_suppression_factor = confidence * target_params.current_suppression_factor +
                                                (1.0 - confidence) * adaptive_params_.current_suppression_factor;

  adaptive_params_.current_heading_weight =
      confidence * target_params.current_heading_weight + (1.0 - confidence) * adaptive_params_.current_heading_weight;

  adaptive_params_.target_heading_weight =
      confidence * target_params.target_heading_weight + (1.0 - confidence) * adaptive_params_.target_heading_weight;

  // 更新滤波参数（根据地形类型立即切换）
  current_alpha_ = terrain_params.alpha;
  current_smoother_frequency_ = terrain_params.smoother_frequency;
  current_smoother_damping_ = terrain_params.smoother_damping;
  alpha_ = current_alpha_;  // 更新当前使用的alpha
  
  // 更新二阶平滑器参数
  angular_smoother_.setParameters(current_smoother_frequency_, current_smoother_damping_);

  // 更新动态调整因子
  terrain_data_.angle_vel_dynamic_factor = 1.0 - confidence * 0.5;  // 置信度越高,抑制越强

  // 记录控制参数变化（保存到文件而不是输出到控制台）
  TerrainControlParams current_params;
  current_params.cross_track_deadzone = adaptive_params_.current_cross_track_deadzone;
  current_params.yaw_deadzone = adaptive_params_.current_yaw_deadzone;
  current_params.max_angular_vel = adaptive_params_.current_max_angular_vel;
  current_params.max_angular_accel = adaptive_params_.current_max_angular_accel;
  current_params.suppression_factor = adaptive_params_.current_suppression_factor;
  current_params.current_heading_weight = adaptive_params_.current_heading_weight;
  current_params.target_heading_weight = adaptive_params_.target_heading_weight;

  std::string change_reason = "地形自适应调整 - 置信度:" + std::to_string(confidence);
}

/**
 * @brief 选择对应地形类型的控制参数
 * @param terrain_type 地形类型
 * @return 对应的控制参数
 */
const LineFollowController::TerrainControlParams&
LineFollowController::selectTerrainParams(TerrainType terrain_type) const
{
  switch (terrain_type)
  {
    case TerrainType::SUDDEN_CHANGE:
      return sudden_change_params_;
    case TerrainType::GENTLE_UNDULATION:
      return gentle_undulation_params_;
    case TerrainType::FREQUENT_BUMPS:
      return frequent_bumps_params_;
    case TerrainType::TRANSITIONING:
      // 过渡状态默认使用平缓凹凸参数，具体插值在adaptControlParameters中处理
      return gentle_undulation_params_;
    case TerrainType::SMOOTH:
    default:
      return smooth_terrain_params_;  // 默认使用平稳地形参数
  }
}

/**
 * @brief 地形类型转换为字符串（用于日志）
 * @param type 地形类型
 * @return 地形类型的字符串表示
 */
std::string LineFollowController::terrainTypeToString(TerrainType type) const
{
  switch (type)
  {
    case TerrainType::SMOOTH:
      return "平稳地形";
    case TerrainType::SUDDEN_CHANGE:
      return "突发变化";
    case TerrainType::GENTLE_UNDULATION:
      return "平缓凹凸";
    case TerrainType::FREQUENT_BUMPS:
      return "频繁颠簸";
    case TerrainType::TRANSITIONING:
      return "过渡状态";
    case TerrainType::UNKNOWN:
      return "未知地形";
    default:
      return "未定义";
  }
}

double LineFollowController::computeVariance(const std::vector<double>& data)
{
  if (data.empty())
    return 0.0;

  // 计算均值
  double mean = 0.0;
  for (double value : data)
  {
    mean += value;
  }
  mean /= data.size();

  // 计算方差
  double variance = 0.0;
  for (double value : data)
  {
    double diff = value - mean;
    variance += diff * diff;
  }
  variance /= data.size();

  return variance;
}

// ================================
// 详细数据记录方法实现
// ================================

void LineFollowController::recordImuTerrainData()
{
  if (!enable_imu_terrain_logging_ || imu_history_.empty())
  {
    return;
  }

  // 计算相对于任务开始的时间戳
  auto current_time = std::chrono::steady_clock::now();
  double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - task_start_time_).count();

  ImuTerrainData data;
  data.timestamp = timestamp;
  data.current_roll = current_roll_angle_;
  data.filtered_roll = filtered_roll_angle_;

  // 新的地形特征数据
  data.max_gradient = terrain_data_.features.max_gradient;
  data.zero_crossing_rate = terrain_data_.features.zero_crossing_rate;
  data.low_freq_energy_ratio = terrain_data_.features.low_freq_energy_ratio;
  data.terrain_type = terrainTypeToString(terrain_data_.current_type);
  data.confidence_score = terrain_data_.confidence_score;

  data.dynamic_factor = terrain_data_.angle_vel_dynamic_factor;
  data.imu_history_size = imu_history_.size();

  // 计算翻滚角统计信息
  calculateRollStatistics(data.roll_mean, data.roll_std_dev, data.roll_max, data.roll_min, data.roll_range);

  // 记录当前使用的地形自适应参数
  data.adaptive_cross_track_deadzone = adaptive_params_.current_cross_track_deadzone;
  data.adaptive_yaw_deadzone = adaptive_params_.current_yaw_deadzone;
  data.adaptive_max_angular_vel = adaptive_params_.current_max_angular_vel;
  data.adaptive_max_angular_accel = adaptive_params_.current_max_angular_accel;
  data.adaptive_suppression_factor = adaptive_params_.current_suppression_factor;
  data.high_precision_active = adaptive_params_.high_precision_active;

  // 添加到数据容器
  imu_terrain_data_.push_back(data);

  static int count = 0;

  if (debug_ && ++count >= 150)
  {
    // LOG_INFO(
        // "IMU地形数据记录 - 时间:{:.3f}s, 地形类型:{}, 置信度:{:.3f}, 特征[梯度:{:.6f} 过零率:{:.2f} 低频比:{:.3f}]",
        // timestamp, data.terrain_type, data.confidence_score, data.max_gradient, data.zero_crossing_rate,
        // data.low_freq_energy_ratio);
    count = 0;
  }
}

void LineFollowController::calculateRollStatistics(double& mean, double& std_dev, double& max_val, double& min_val,
                                                   double& range)
{
  if (imu_history_.empty())
  {
    mean = std_dev = max_val = min_val = range = 0.0;
    return;
  }

  // 计算均值
  mean = 0.0;
  max_val = std::numeric_limits<double>::lowest();
  min_val = std::numeric_limits<double>::max();

  for (const auto& point : imu_history_)
  {
    mean += point.roll;
    max_val = std::max(max_val, point.roll);
    min_val = std::min(min_val, point.roll);
  }
  mean /= imu_history_.size();

  // 计算标准差
  double variance = 0.0;
  for (const auto& point : imu_history_)
  {
    double diff = point.roll - mean;
    variance += diff * diff;
  }
  variance /= imu_history_.size();
  std_dev = std::sqrt(variance);

  // 计算范围
  range = max_val - min_val;
}

// 控制效果评估方法
double LineFollowController::calculateControlStabilityScore(double current_angular_vel, double prev_angular_vel)
{
  // 基于角速度变化率计算稳定性得分 (0-100)
  double angular_vel_change = std::abs(current_angular_vel - prev_angular_vel);
  double max_expected_change = 0.5;  // 最大预期变化率

  // 变化率越小,稳定性得分越高
  double score = std::max(0.0, 100.0 * (1.0 - angular_vel_change / max_expected_change));
  return std::min(100.0, score);
}

double LineFollowController::calculateTrackingAccuracyScore(double cross_track_error)
{
  // 基于横向误差计算跟踪精度得分 (0-100)
  double max_acceptable_error = 0.05;  // 5cm最大可接受误差
  double abs_error = std::abs(cross_track_error);

  // 误差越小,精度得分越高
  double score = std::max(0.0, 100.0 * (1.0 - abs_error / max_acceptable_error));
  return std::min(100.0, score);
}

double LineFollowController::calculateResponseTimeScore(double yaw_error, double angular_response)
{
  // 基于响应时间计算得分 (0-100)
  if (std::abs(yaw_error) < 0.01)
  {  // 如果误差很小,认为响应良好
    return 100.0;
  }

  // 计算响应比率（角速度/角度误差）
  double response_ratio = std::abs(angular_response) / std::abs(yaw_error);
  double ideal_response_ratio = 2.0;  // 理想响应比率

  // 响应比率越接近理想值,得分越高
  double score = 100.0 * std::exp(-std::abs(response_ratio - ideal_response_ratio) / ideal_response_ratio);
  return std::min(100.0, score);
}

void LineFollowController::exportImuTerrainData()
{
  if (imu_terrain_data_.empty())
  {
    return;
  }

  // 创建输出目录
  std::filesystem::create_directories(data_log_base_path_);

  // 生成文件名
  std::string filename = data_log_base_path_ + "task_" + current_task_id_ + "_imu_terrain.csv";

  // 写入CSV文件
  std::ofstream file(filename);
  if (!file.is_open())
  {
    LOG_ERROR("无法创建IMU地形数据文件: {}", filename);
    return;
  }

  // 写入CSV头（三场景版本）
  file << "timestamp,current_roll,filtered_roll,max_gradient,zero_crossing_rate,"
       << "low_freq_energy_ratio,terrain_type,confidence_score,dynamic_factor,"
       << "imu_history_size,roll_mean,roll_std_dev,"
       << "roll_max,roll_min,roll_range,adaptive_cross_track_deadzone,"
       << "adaptive_yaw_deadzone,adaptive_max_angular_vel,adaptive_max_angular_accel,"
       << "adaptive_suppression_factor,high_precision_active\n";

  // 写入数据
  for (const auto& data : imu_terrain_data_)
  {
    file << std::fixed << std::setprecision(6) << data.timestamp << "," << data.current_roll << ","
         << data.filtered_roll << "," << data.max_gradient << "," << data.zero_crossing_rate << ","
         << data.low_freq_energy_ratio << "," << data.terrain_type << "," << data.confidence_score << ","
         << data.dynamic_factor << "," << data.imu_history_size << "," << data.roll_mean << "," << data.roll_std_dev
         << "," << data.roll_max << "," << data.roll_min << "," << data.roll_range << ","
         << data.adaptive_cross_track_deadzone << "," << data.adaptive_yaw_deadzone << ","
         << data.adaptive_max_angular_vel << "," << data.adaptive_max_angular_accel << ","
         << data.adaptive_suppression_factor << "," << (data.high_precision_active ? "1" : "0") << "\n";
  }

  file.close();
  LOG_INFO("IMU地形数据已导出至: {}", filename);

  // 清空数据容器
  imu_terrain_data_.clear();
}

/**
 * @brief 立即地形管理（无过渡控制）
 * @param features 地形特征数据
 * @return 检测到的地形类型
 */
LineFollowController::TerrainType
LineFollowController::manageTerrainImmediate(const LineFollowController::TerrainFeatures& features)
{
  // 立即切换模式：直接进行地形分类，无任何过渡控制
  TerrainType current_terrain = classifyTerrainWithPriority(features);

  // 记录状态变化（如果有变化）
  if (current_terrain != terrain_data_.current_type)
  {

    // 更新当前地形类型
    terrain_data_.current_type = current_terrain;
    state_memory_.terrain_change_count++;
  }

  return current_terrain;
}

/**
 * @brief 更新状态记忆（立即切换模式简化版）
 */
void LineFollowController::updateStateMemory(TerrainType current_terrain)
{
  state_memory_.previous_terrain = current_terrain;
  state_memory_.last_terrain_change_time = std::chrono::steady_clock::now();
}


/**
 * @brief 初始化地形日志发布器
 */
void LineFollowController::initializeTerrainLogPublisher()
{
  try
  {
    // 使用IMU节点作为发布节点（复用现有节点）
    if (imu_node_)
    {
      terrain_log_publisher_ =
          imu_node_->create_publisher<std_msgs::msg::String>(terrain_log_topic_name_, rclcpp::QoS(10).reliable());
      LOG_INFO("地形日志发布器已初始化，话题: {}", terrain_log_topic_name_);
    }
    else
    {
      LOG_ERROR("无法初始化地形日志发布器: IMU节点未初始化");
    }
  }
  catch (const std::exception& e)
  {
    LOG_ERROR("初始化地形日志发布器失败: {}", e.what());
  }
}

/**
 * @brief 发布地形日志数据
 */
void LineFollowController::publishTerrainLogData(const TerrainFeatures& features, TerrainType classified_type,
                                                 double confidence)
{
  if (!terrain_log_publisher_)
  {
    return;  // 发布器未初始化，直接返回
  }

  try
  {
    // 创建JSON消息
    std::string json_message = createTerrainJsonMessage(features, classified_type, confidence);

    // 发布消息
    auto msg = std_msgs::msg::String();
    msg.data = json_message;
    terrain_log_publisher_->publish(msg);
  }
  catch (const std::exception& e)
  {
    LOG_ERROR("发布地形日志数据失败: {}", e.what());
  }
}

/**
 * @brief 发布控制日志数据
 */
void LineFollowController::publishControlLogData(double yaw_error, double cross_track_error, 
                                                const geometry_msgs::msg::TwistStamped& cmd_vel,
                                                bool back_follow, bool in_deadzone, double deadzone_factor, 
                                                double d_adaptive_factor, double max_angular_vel, double suppression_factor,
                                                double cross_track_deadzone, double yaw_deadzone, double max_angular_accel,
                                                double current_angular_accel, bool imu_terrain_enabled)
{
  if (!terrain_log_publisher_)
  {
    return;  // 发布器未初始化，直接返回
  }

  try
  {
    // 创建JSON消息
    std::string json_message = createControlJsonMessage(yaw_error, cross_track_error, cmd_vel, back_follow, 
                                                       in_deadzone, deadzone_factor, d_adaptive_factor,
                                                       max_angular_vel, suppression_factor, cross_track_deadzone,
                                                       yaw_deadzone, max_angular_accel, current_angular_accel,
                                                       imu_terrain_enabled);

    // 发布消息
    auto msg = std_msgs::msg::String();
    msg.data = json_message;
    terrain_log_publisher_->publish(msg);
  }
  catch (const std::exception& e)
  {
    LOG_ERROR("发布控制日志数据失败: {}", e.what());
  }
}

/**
 * @brief 创建地形分类JSON消息
 */
std::string LineFollowController::createTerrainJsonMessage(const TerrainFeatures& features, TerrainType classified_type,
                                                           double confidence)
{
  std::stringstream json_ss;
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  // 格式化时间戳
  json_ss << std::fixed << std::setprecision(3);
  std::stringstream time_ss;
  time_ss << std::put_time(std::localtime(&time_t), "%Y-%m-%dT%H:%M:%S");
  time_ss << "." << std::setfill('0') << std::setw(3) << ms.count();

  // 构建JSON消息
  json_ss << "{\n";
  json_ss << "  \"timestamp\": \"" << time_ss.str() << "\",\n";
  json_ss << "  \"terrain_classification\": {\n";
  json_ss << "    \"current_type\": \"" << terrainTypeToString(classified_type) << "\",\n";
  json_ss << "    \"features\": {\n";
  json_ss << "      \"max_gradient\": " << std::fixed << std::setprecision(6) << features.max_gradient << ",\n";
  json_ss << "      \"low_freq_energy_ratio\": " << std::fixed << std::setprecision(6) << features.low_freq_energy_ratio
          << ",\n";
  json_ss << "      \"zero_crossing_rate\": " << std::fixed << std::setprecision(6) << features.zero_crossing_rate
          << "\n";
  json_ss << "    },\n";
  json_ss << "    \"thresholds\": {\n";
  json_ss << "      \"sudden_change_gradient\": " << std::fixed << std::setprecision(6)
          << sudden_change_gradient_threshold_ << ",\n";
  json_ss << "      \"gentle_low_freq_energy\": " << std::fixed << std::setprecision(6)
          << gentle_low_freq_energy_threshold_ << ",\n";
  json_ss << "      \"frequent_zero_crossing\": " << std::fixed << std::setprecision(6)
          << frequent_zero_crossing_threshold_ << "\n";
  json_ss << "    },\n";
  json_ss << "    \"analysis\": {\n";
  json_ss << "      \"low_freq_energy_comparison\": \"" << std::fixed << std::setprecision(6)
          << features.low_freq_energy_ratio << " "
          << (features.low_freq_energy_ratio > gentle_low_freq_energy_threshold_ ? ">" : "<=") << " "
          << gentle_low_freq_energy_threshold_ << "\",\n";
  json_ss << "      \"max_gradient_comparison\": \"" << std::fixed << std::setprecision(6) << features.max_gradient
          << " " << (features.max_gradient > sudden_change_gradient_threshold_ ? ">" : "<=") << " "
          << sudden_change_gradient_threshold_ << "\",\n";
  json_ss << "      \"zero_crossing_comparison\": \"" << std::fixed << std::setprecision(2)
          << features.zero_crossing_rate << " "
          << (features.zero_crossing_rate > frequent_zero_crossing_threshold_ ? ">" : "<=") << " "
          << frequent_zero_crossing_threshold_ << "\",\n";
  json_ss << "      \"low_freq_energy_desc\": \"低频能量占比\",\n";
  json_ss << "      \"max_gradient_desc\": \"最大梯度\",\n";
  json_ss << "      \"zero_crossing_desc\": \"过零率\"\n";
  json_ss << "    },\n";
  json_ss << "    \"dynamic_factors\": {\n";
  json_ss << "      \"line_vel_factor\": " << std::fixed << std::setprecision(2)
          << terrain_data_.line_vel_dynamic_factor << ",\n";
  json_ss << "      \"angle_vel_factor\": " << std::fixed << std::setprecision(2)
          << terrain_data_.angle_vel_dynamic_factor << "\n";
  json_ss << "    },\n";
  json_ss << "    \"confidence\": " << std::fixed << std::setprecision(3) << confidence << "\n";
  json_ss << "  }\n";
  json_ss << "}";

  return json_ss.str();
}

/**
 * @brief 创建控制日志JSON消息
 */
std::string LineFollowController::createControlJsonMessage(double yaw_error, double cross_track_error, 
                                                          const geometry_msgs::msg::TwistStamped& cmd_vel,
                                                          bool back_follow, bool in_deadzone, double deadzone_factor, 
                                                          double d_adaptive_factor, double max_angular_vel, double suppression_factor,
                                                          double cross_track_deadzone, double yaw_deadzone, double max_angular_accel,
                                                          double current_angular_accel, bool imu_terrain_enabled)
{
  std::stringstream json_ss;
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  // 格式化时间戳
  json_ss << std::fixed << std::setprecision(3);
  std::stringstream time_ss;
  time_ss << std::put_time(std::localtime(&time_t), "%Y-%m-%dT%H:%M:%S");
  time_ss << "." << std::setfill('0') << std::setw(3) << ms.count();

  // 构建控制日志JSON消息
  json_ss << "{\n";
  json_ss << "  \"timestamp\": \"" << time_ss.str() << "\",\n";
  json_ss << "  \"control_data\": {\n";
  json_ss << "    \"path_following\": {\n";
  json_ss << "      \"yaw_error\": " << std::fixed << std::setprecision(6) << yaw_error << ",\n";
  json_ss << "      \"cross_track_error\": " << std::fixed << std::setprecision(6) << cross_track_error << ",\n";
  json_ss << "      \"linear_velocity\": " << std::fixed << std::setprecision(6) << cmd_vel.twist.linear.x << ",\n";
  json_ss << "      \"angular_velocity\": " << std::fixed << std::setprecision(6) << cmd_vel.twist.angular.z << ",\n";
  json_ss << "      \"back_follow\": " << (back_follow ? "true" : "false") << "\n";
  json_ss << "    },\n";
  json_ss << "    \"terrain_adaptation\": {\n";
  json_ss << "      \"imu_terrain_enabled\": " << (imu_terrain_enabled ? "true" : "false") << ",\n";
  json_ss << "      \"in_deadzone\": " << (in_deadzone ? "true" : "false") << ",\n";
  json_ss << "      \"deadzone_factor\": " << std::fixed << std::setprecision(6) << deadzone_factor << ",\n";
  json_ss << "      \"d_adaptive_factor\": " << std::fixed << std::setprecision(6) << d_adaptive_factor << ",\n";
  json_ss << "      \"cross_track_deadzone\": " << std::fixed << std::setprecision(6) << cross_track_deadzone << ",\n";
  json_ss << "      \"yaw_deadzone\": " << std::fixed << std::setprecision(6) << yaw_deadzone << ",\n";
  json_ss << "      \"current_max_angular_vel\": " << std::fixed << std::setprecision(6) << max_angular_vel << ",\n";
  json_ss << "      \"max_angular_accel\": " << std::fixed << std::setprecision(6) << max_angular_accel << ",\n";
  json_ss << "      \"current_angular_accel\": " << std::fixed << std::setprecision(6) << current_angular_accel << ",\n";
  json_ss << "      \"suppression_factor\": " << std::fixed << std::setprecision(6) << suppression_factor << "\n";
  json_ss << "    }\n";
  json_ss << "  }\n";
  json_ss << "}";

  return json_ss.str();
}

}  // namespace follow_controller
}  // namespace xline
