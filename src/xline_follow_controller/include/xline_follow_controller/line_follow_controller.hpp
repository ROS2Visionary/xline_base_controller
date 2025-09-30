#pragma once

#include "xline_follow_controller/base_follow_controller.hpp"
#include "xline_follow_controller/follow_common.hpp"
#include "xline_follow_controller/logging_compat.hpp"
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <deque>
#include <memory>
#include <chrono>
#include <fstream>
#include <mutex>
#include <thread>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <cmath>        // for std::cos, std::sin, M_PI
#include <algorithm>    // for std::clamp
#include <std_msgs/msg/string.hpp>

namespace xline
{
namespace follow_controller
{

/**
 * @brief 直线路径跟随控制器
 *
 * 该控制器实现了机器人沿直线路径的跟随功能，包括：
 * - 初始航向对齐
 * - 路径跟随
 * - 终点航向对齐
 * - 速度平滑控制
 * - 支持前进和后退模式
 */
class LineFollowController : public BaseFollowController
{
public:
  LineFollowController();
  ~LineFollowController();

  // 基类接口实现
  bool setPlan(const nav_msgs::msg::Path& orig_global_plan) override;
  bool computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
                               geometry_msgs::msg::TwistStamped& cmd_vel) override;
  bool isGoalReached() override;
  bool cancel() override;

  // 扩展接口
  // 通过起点/终点坐标直接生成直线路径
  bool setPlan(double start_x, double start_y, double end_x, double end_y);
  bool setPlan(const std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped>>& plan);
  void setSpeedLimit(const double& speed_limit);
  void setWorkState(bool state);
  void setBackFollow(bool back);
  void setPose(const geometry_msgs::msg::PoseStamped& pose);

  void startImuThread();  // 启动IMU处理线程

  // 地形控制参数结构体（公有访问，用于函数返回类型）
  struct TerrainControlParams
  {
    double cross_track_deadzone;
    double yaw_deadzone;
    double max_angular_vel;
    double max_angular_accel;
    double suppression_factor;
    // 航向混合控制参数
    double current_heading_weight;    // 当前机器人航向权重
    double target_heading_weight;     // 目标路径航向权重
    // 角速度滤波参数
    double alpha;                     // 低通滤波器平滑因子（应用于角速度）
    // 二阶平滑器参数
    double smoother_frequency;        // 自然频率
    double smoother_damping;          // 阻尼比
  };

private:
  // ================================
  // 状态管理
  // ================================

  /**
   * @brief 控制器状态枚举
   */
  enum class ControlState
  {
    IDLE,            // 空闲状态
    ALIGNING_START,  // 对齐起始航向
    FOLLOWING_PATH,  // 跟随路径
    ALIGNING_END,    // 对齐终点航向
    GOAL_REACHED,    // 到达目标
    WAITING          // 等待状态
  };

  ControlState current_state_;  // 当前控制器状态
  bool received_plan_;          // 是否接收到路径
  bool goal_reached_;           // 是否到达目标
  bool back_follow_;            // 是否后退模式
  bool m_work_state_;           // 工作状态（影响最大速度）
  bool short_path_;             // 是否为短路径
  bool reset_required_;         // 是否需要重置
  bool debug_;                  // 调试模式开关
  bool decel_phase_entered_;    // 是否已进入减速阶段

  // ================================
  // 路径和位姿信息
  // ================================

  nav_msgs::msg::Path global_plan_;                       // 全局路径
  geometry_msgs::msg::PoseStamped start_pose_;            // 起始位姿
  geometry_msgs::msg::PoseStamped target_pose_;           // 目标位姿
  geometry_msgs::msg::PoseStamped end_pose_;              // 终点位姿
  geometry_msgs::msg::PoseStamped current_pose_;          // 当前位姿
  geometry_msgs::msg::PoseStamped original_target_pose_;  // 原始目标位姿（延长前）

  size_t current_waypoint_index_;  // 当前路径点索引
  double path_length_;             // 路径长度
  double original_path_length_;    // 原始路径长度（延长前）
  double robot_yaw_;               // 机器人当前偏航角

  // 路径延长参数
  static constexpr double PATH_EXTENSION_LENGTH = 0.5;  // 路径延长距离（米）
  static constexpr double PATH_POINT_INTERVAL = 0.001;   // 路径点间隔（米）

  // ================================
  // 速度控制参数
  // ================================

  // 线速度参数
  double max_linear_speed_;      // 最大线速度
  double min_linear_speed_;      // 最小线速度
  double current_linear_speed_;  // 当前线速度
  double m_walk_max_vel_;        // 步行最大速度
  double m_work_max_vel_;        // 工作最大速度
  double m_alignment_vel_;       // 对齐阶段速度
  double mini_path_distance_;

  // 角速度参数
  double max_rotation_angular_vel_;        // 最大角速度
  double min_rotation_angular_vel_;        // 最小角速度
  double current_angular_speed_;  // 当前角速度
  double rotation_angular_factor_;         // 角速度调节因子
  double rotation_angle_threshold_;        // 角度阈值
  double rotation_angle_smooth_factor_;          // 平滑因子

  // IMU基于颠簸检测的动态控制参数
  bool enable_imu_terrain_adaptation_;  // 是否启用基于IMU的地形自适应控制
  double imu_window_size_;              // IMU数据滑动窗口大小（秒）
  // 三场景地形检测阈值参数（基于航空级IMU精度）
  double sudden_change_gradient_threshold_;  // 突变检测：最大梯度阈值（rad/sample）
  double gentle_low_freq_energy_threshold_;  // 平缓凹凸：低频能量阈值（0-1）
  double frequent_zero_crossing_threshold_;  // 频繁颠簸：过零率阈值（次数/秒）

  // 地形类型枚举（按优先级排序）
  enum class TerrainType
  {
    SMOOTH = 0,             // 平稳地形（基础状态）
    SUDDEN_CHANGE = 1,      // 突发变化（最高优先级，立即响应）
    GENTLE_UNDULATION = 2,  // 平缓凹凸（中优先级，可升级基础地形）
    FREQUENT_BUMPS = 3,     // 频繁颠簸（低优先级）
    TRANSITIONING = 4,      // 过渡状态（渐进切换中）
    UNKNOWN = 5             // 未知状态（初始化或错误状态）
  };
  
  // 地形控制配置结构（立即切换模式）
  struct TerrainControlConfig
  {
    bool immediate_switch = true;  // 立即切换模式
  };
    
  // 地形状态记录结构（简化版）
  struct TerrainStateMemory
  {
    TerrainType previous_terrain = TerrainType::SMOOTH;             // 前一次地形类型
    std::chrono::steady_clock::time_point last_terrain_change_time; // 最后地形变化时间
    int terrain_change_count = 0;                                   // 地形变化总次数
    int sudden_change_count = 0;                                    // 突变检测次数
  };
  

  // 地形特征数据结构（基于航空级IMU精度优化）
  struct TerrainFeatures
  {
    double max_gradient = 0.0;           // 最大梯度：突变检测
    double zero_crossing_rate = 0.0;     // 过零率：频繁颠簸检测
    double low_freq_energy_ratio = 0.0;  // 低频能量占比：平缓凹凸检测
    std::chrono::steady_clock::time_point last_update;
  };

  // 地形分析数据结构（三场景版本）
  struct TerrainAnalysisData
  {
    TerrainType current_type = TerrainType::SMOOTH;  // 当前地形类型
    TerrainFeatures features;                        // 地形特征
    double confidence_score = 0.0;                   // 分类置信度
    double angle_vel_dynamic_factor = 1.0;                     // 角速度动态调整因子
    double line_vel_dynamic_factor = 1.0;                     // 线速度动态调整因子
    std::chrono::steady_clock::time_point last_update;
  };

  TerrainAnalysisData terrain_data_;  // 地形分析数据（新版）
  
  // 地形控制成员变量
  TerrainControlConfig terrain_config_;   // 地形控制配置
  TerrainStateMemory state_memory_;       // 地形状态记忆
  
  // 水泥地面模式相关变量
  double current_pitch_angle_ = 0.0;           // 当前俯仰角
  double current_yaw_rate_ = 0.0;              // 当前偏航率

  // IMU历史数据存储（增强版 - 存储翻滚角和俯仰角用于突变检测）
  struct ImuDataPoint
  {
    double roll;    // 翻滚角：检测侧向地形变化
    double pitch;   // 俯仰角：检测前向地形变化（突变检测关键）
    std::chrono::steady_clock::time_point timestamp;
  };

  std::deque<ImuDataPoint> imu_history_;  // IMU数据历史队列
  size_t max_imu_history_size_;           // 最大历史数据数量

  // 动态调整的控制参数
  struct AdaptiveControlParams
  {
    // 跟随阶段参数 (distance_to_start > alignment_distance)
    double current_cross_track_deadzone = 0.006;  // 当前横向误差死区
    double current_yaw_deadzone = 0.02;           // 当前航向误差死区
    double current_max_angular_vel = 0.05;        // 当前最大角速度
    double current_max_angular_accel = 0.1;       // 当前最大角加速度
    double current_suppression_factor = 0.5;      // 当前抑制因子
    // 航向混合控制参数
    double current_heading_weight = 0.0;          // 当前机器人航向权重
    double target_heading_weight = 1.0;           // 目标路径航向权重

    bool high_precision_active = true;  // 高精度模式是否激活
  };

  AdaptiveControlParams adaptive_params_;  // 自适应控制参数

  



  // 地形控制参数结构体已移至public部分

  // 三种场景地形控制参数
  TerrainControlParams sudden_change_params_;      // 突发变化地形参数
  TerrainControlParams gentle_undulation_params_;  // 平缓凹凸地形参数
  TerrainControlParams frequent_bumps_params_;     // 频繁颠簸地形参数
  TerrainControlParams smooth_terrain_params_;     // 平稳地形参数（基线）
  TerrainControlParams alignment_params_;          // 通用对齐阶段参数

  // 翻滚角数据（用于调试和记录）
  double current_roll_angle_;   // 当前翻滚角
  double filtered_roll_angle_;  // 滤波后翻滚角

  // ================================
  // 运动控制参数
  // ================================

  // 距离参数
  double waypoint_tolerance_;     // 路径点容差
  double yaw_tolerance_;          // 角度容差
  double lookahead_distance_;     // 前瞻距离
  double deceleration_distance_;  // 减速距离
  double m_alignment_distance_;   // 对齐距离
  double m_acce_distance_;        // 加速距离

  // 速度调节参数
  double m_acce_factor_;                  // 加速因子
  double m_acceleration_factor_;          // 加速sigmoid因子
  double m_deceleration_factor_;          // 减速sigmoid因子
  double m_acceleration_sigmoid_center_;  // 加速sigmoid中心点
  double m_deceleration_sigmoid_center_;  // 减速sigmoid中心点
  double max_angular_acceleration_;       // 最大角加速度

  // ================================
  // PID控制器
  // ================================

  std::shared_ptr<PIDController> pid_heading_controller_;  // 航向PID控制器
  double angular_kp_;                                      // 比例系数
  double angular_ki_;                                      // 积分系数
  double angular_kd_;                                      // 微分系数

  // ================================
  // 滤波器
  // ================================

  // 位置滤波
  SavitzkyGolayFilter x_filter_;  // X坐标SG滤波器
  SavitzkyGolayFilter y_filter_;  // Y坐标SG滤波器
  HampelFilter h_x_filter_;       // X坐标Hampel滤波器
  HampelFilter h_y_filter_;       // Y坐标Hampel滤波器

  int m_window_size_;       // SG滤波窗口大小
  int m_polynomial_order_;  // SG多项式阶数
  int m_hampel_window_;     // Hampel滤波窗口
  double m_hampel_k_;       // Hampel滤波参数

  // 角速度滤波
  double prev_angular_velocity_;           // 上一次角速度
  double prev_smoothed_angular_velocity_;  // 上一次平滑后角速度
  double last_yaw_error_;                  // 上一次航向误差
  double second_prev_angular_velocity_;
  double alpha_;                           // 当前使用的低通滤波器平滑因子
  std::deque<double> angular_vel_history_;  // 角速度历史记录
  int angular_vel_history_size_ = 5;
  HampelFilter angular_vel_hampel_filter_;  // 角速度Hampel滤波器
  int angular_following_hampel_window_;     
  double angular_following_hampel_k_;       
  
  // 当前地形动态滤波参数（会根据地形类型动态切换）
  double current_alpha_;                   // 当前地形使用的alpha参数
  double current_smoother_frequency_;      // 当前地形使用的二阶平滑器频率
  double current_smoother_damping_;        // 当前地形使用的二阶平滑器阻尼

  // ================================
  // 虚拟位置跟踪器
  // ================================

  /**
   * @brief 虚拟位置跟踪器类 - 超高精度直线跟随
   * 
   * 通过运动学模型预测位置，使用全站仪数据进行适度修正，
   * 实现5mm精度的直线路径跟随
   */
  class VirtualPositionTracker
  {
  public:
    VirtualPositionTracker() 
      : virtual_x_(0.0), virtual_y_(0.0), virtual_yaw_(0.0)
      , initialized_(false), last_update_time_(std::chrono::steady_clock::now())
      , correction_gain_(0.15), max_correction_(0.005)
      , position_trust_threshold_(0.02), max_velocity_trust_(0.5)
      , angular_velocity_factor_(1.0)
      , smooth_gain_(0.25), rough_gain_(0.08), sudden_change_gain_(0.05)
      , debug_logging_(false), log_frequency_(50), log_counter_(0)
    {
      // 默认参数将在updateParameters()中从配置文件加载
    }

    /**
     * @brief 初始化虚拟位置
     */
    void initialize(double x, double y, double yaw)
    {
      virtual_x_ = x;
      virtual_y_ = y;
      virtual_yaw_ = yaw;
      initialized_ = true;
      last_update_time_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief 更新虚拟位置（基于运动学模型）
     */
    void updateVirtualPose(double linear_vel, double angular_vel, double dt)
    {
      if (!initialized_) return;
      
      // 速度约束检查
      if (std::abs(linear_vel) > max_velocity_trust_) {
        linear_vel = std::copysign(max_velocity_trust_, linear_vel);
      }
      
      // 运动学模型更新
      double dx = linear_vel * std::cos(virtual_yaw_) * dt;
      double dy = linear_vel * std::sin(virtual_yaw_) * dt;
      
      virtual_x_ += dx;
      virtual_y_ += dy;
      virtual_yaw_ += angular_vel * angular_velocity_factor_ * dt;
      
      // 航向角归一化
      while (virtual_yaw_ > M_PI) virtual_yaw_ -= 2.0 * M_PI;
      while (virtual_yaw_ < -M_PI) virtual_yaw_ += 2.0 * M_PI;
      
      last_update_time_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief 使用全站仪数据修正虚拟位置
     */
    void correctWithMeasurement(double measured_x, double measured_y, TerrainType terrain_type)
    {
      if (!initialized_) {
        initialize(measured_x, measured_y, 0.0);
        return;
      }
      
      // 计算误差
      double error_x = measured_x - virtual_x_;
      double error_y = measured_y - virtual_y_;
      double position_error = std::sqrt(error_x * error_x + error_y * error_y);
      
      // 根据地形选择修正增益
      double adaptive_gain = getAdaptiveGain(terrain_type);
      
      // 如果误差过大，可能是全站仪数据异常，减少修正
      if (position_error > position_trust_threshold_) {
        adaptive_gain *= 0.3;  // 大误差时减少信任度
      }
      
      // 限制修正量（5mm超高精度）
      double correction_x = std::clamp(error_x, -max_correction_, max_correction_);
      double correction_y = std::clamp(error_y, -max_correction_, max_correction_);
      
      // 应用修正
      virtual_x_ += adaptive_gain * correction_x;
      virtual_y_ += adaptive_gain * correction_y;
      
      // 调试日志
      if (debug_logging_ && (++log_counter_ % log_frequency_ == 0)) {
        logTrackingData(measured_x, measured_y, error_x, error_y, adaptive_gain, terrain_type);
      }
    }

    // Getter方法
    double getVirtualX() const { return virtual_x_; }
    double getVirtualY() const { return virtual_y_; }
    double getVirtualYaw() const { return virtual_yaw_; }
    bool isInitialized() const { return initialized_; }

    // 参数设置方法
    void setParameters(double correction_gain, double max_correction, double position_trust_threshold,
                      double max_velocity_trust, double angular_velocity_factor)
    {
      correction_gain_ = correction_gain;
      max_correction_ = max_correction;
      position_trust_threshold_ = position_trust_threshold;
      max_velocity_trust_ = max_velocity_trust;
      angular_velocity_factor_ = angular_velocity_factor;
    }

    void setTerrainGains(double smooth_gain, double rough_gain, double sudden_change_gain)
    {
      smooth_gain_ = smooth_gain;
      rough_gain_ = rough_gain;
      sudden_change_gain_ = sudden_change_gain;
    }

    void setDebugLogging(bool enabled, int frequency)
    {
      debug_logging_ = enabled;
      log_frequency_ = frequency;
    }
    
    /**
     * @brief 重置跟踪器状态（用于新路径开始）
     */
    void reset()
    {
      virtual_x_ = 0.0;
      virtual_y_ = 0.0;
      virtual_yaw_ = 0.0;
      initialized_ = false;
      last_update_time_ = std::chrono::steady_clock::now();
      log_counter_ = 0;
    }

  private:
    // 虚拟位置状态
    double virtual_x_, virtual_y_, virtual_yaw_;
    bool initialized_;
    std::chrono::steady_clock::time_point last_update_time_;
    
    // 控制参数
    double correction_gain_;           // 基础修正增益
    double max_correction_;            // 最大修正量
    double position_trust_threshold_;  // 位置信任阈值
    double max_velocity_trust_;        // 最大可信速度
    double angular_velocity_factor_;   // 角速度影响因子
    
    // 地形自适应增益
    double smooth_gain_;               // 平稳地形增益
    double rough_gain_;                // 不平整地形增益
    double sudden_change_gain_;        // 突变地形增益
    
    // 调试相关
    bool debug_logging_;
    int log_frequency_;
    int log_counter_;

    /**
     * @brief 根据地形获取自适应修正增益
     */
    double getAdaptiveGain(TerrainType terrain_type)
    {
      switch(terrain_type) {
        case TerrainType::SMOOTH:
          return smooth_gain_;
        case TerrainType::GENTLE_UNDULATION:
        case TerrainType::FREQUENT_BUMPS:
          return rough_gain_;
        case TerrainType::SUDDEN_CHANGE:
          return sudden_change_gain_;
        default:
          return correction_gain_;  // 默认增益
      }
    }

    /**
     * @brief 记录跟踪调试数据
     */
    void logTrackingData(double measured_x, double measured_y, double error_x, double error_y,
                        double gain, TerrainType terrain_type)
    {
      // 避免未使用参数的编译器告警
      (void)measured_x; (void)measured_y; (void)error_x; (void)error_y; (void)gain; (void)terrain_type;
      // 可以在这里添加详细的日志记录逻辑
      // 例如写入文件或发布到ROS话题
    }
  };

  VirtualPositionTracker virtual_tracker_;  // 虚拟位置跟踪器实例
  
  // 虚拟跟踪配置参数（所有参数从line.yaml配置文件读取）
  struct VirtualTrackingConfig {
    bool enabled = true;                     // sensors.position.virtual_tracking.enabled
    double correction_gain = 0.15;           // sensors.position.virtual_tracking.correction_gain
    double max_correction = 0.005;           // sensors.position.virtual_tracking.max_correction (5mm超高精度)
    double position_trust_threshold = 0.02;  // sensors.position.virtual_tracking.position_trust_threshold (2cm)
    double max_velocity_trust = 0.5;         // sensors.position.virtual_tracking.kinematics.max_velocity_trust
    double angular_velocity_factor = 1.0;    // sensors.position.virtual_tracking.kinematics.angular_velocity_factor
    double smooth_gain = 0.25;               // sensors.position.virtual_tracking.terrain_adaptive.smooth_gain
    double rough_gain = 0.08;                // sensors.position.virtual_tracking.terrain_adaptive.rough_gain
    double sudden_change_gain = 0.05;        // sensors.position.virtual_tracking.terrain_adaptive.sudden_change_gain
    bool debug_logging = false;              // sensors.position.virtual_tracking.debug.enable_logging
    int log_frequency = 50;                  // sensors.position.virtual_tracking.debug.log_frequency
  } virtual_tracking_config_;

  // ================================
  // 时间管理
  // ================================

  std::chrono::steady_clock::time_point last_time_;        // 上次计算时间
  std::chrono::steady_clock::time_point wait_start_time_;  // 等待开始时间
  double wait_duration_;                                   // 等待持续时间
  bool waiting_;                                           // 是否在等待

  // ================================
  // 调试和数据记录
  // ================================

  struct DebugInfo
  {
    double target_speed;
    double distance_to_target;
    double distance_to_start;
    double cross_track_error;
    double yaw_error;
    double yaw_pid_output;
    double filtered_pos_x;
    double filtered_pos_y;
    std::chrono::steady_clock::time_point timestamp;
  };

  // IMU地形数据记录结构（扩展版 - 支持三种场景）
  struct ImuTerrainData
  {
    double timestamp;      // 时间戳（秒）
    double current_roll;   // 当前翻滚角（rad）
    double filtered_roll;  // 滤波后翻滚角（rad）

    // 新的地形特征数据
    double max_gradient;           // 最大梯度（突变检测）
    double zero_crossing_rate;     // 过零率（频繁颠簸检测）
    double low_freq_energy_ratio;  // 低频能量占比（平缓凹凸检测）
    std::string terrain_type;      // 地形类型（突发变化/平缓凹凸/频繁颠簸/平稳）
    double confidence_score;       // 分类置信度（0-1）

    double dynamic_factor;  // 动态调整因子

    // 统计信息
    size_t imu_history_size;  // 当前IMU历史数据数量
    double roll_mean;         // 翻滚角均值
    double roll_std_dev;      // 翻滚角标准差
    double roll_max;          // 窗口内最大翻滚角
    double roll_min;          // 窗口内最小翻滚角
    double roll_range;        // 翻滚角变化范围

    // 地形自适应控制参数（当前使用的）
    double adaptive_cross_track_deadzone;  // 自适应横向死区
    double adaptive_yaw_deadzone;          // 自适应航向死区
    double adaptive_max_angular_vel;       // 自适应最大角速度
    double adaptive_max_angular_accel;     // 自适应最大角加速度
    double adaptive_suppression_factor;    // 自适应抑制因子
    bool high_precision_active;            // 高精度模式是否激活
  };




  std::vector<std::vector<double>> original_path_;         // 原始路径点
  std::vector<std::vector<double>> filtered_path_;         // 滤波后路径点
  std::vector<ImuTerrainData> imu_terrain_data_;           // IMU地形数据记录
  std::mutex file_mutex_;                                  // 文件操作互斥锁
  std::string current_task_id_;                            // 当前任务ID
  std::chrono::steady_clock::time_point task_start_time_;  // 任务开始时间

  // 可配置的数据记录参数
  std::string data_log_base_path_;                       // 数据日志基础路径（可配置）
  bool enable_detailed_logging_;                         // 是否启用详细日志记录
  bool enable_imu_terrain_logging_;                      // 是否启用IMU地形数据记录
  double logging_frequency_;                             // 日志记录频率（Hz）
  std::chrono::steady_clock::time_point last_log_time_;  // 上次记录时间

  // 二阶平滑器相关参数
  /*
  自然频率 (ωn)：控制系统响应速度，值越大响应越快
  阻尼比 (ζ)：控制震荡程度
  ζ < 1: 欠阻尼（有震荡）
  ζ = 1: 临界阻尼（最快无震荡响应）
  ζ > 1: 过阻尼（响应较慢但很平滑）
  */
  double angular_smoother_freq_;     // 自然频率，默认2.0 Hz
  double angular_smoother_damping_;  // 阻尼比，默认0.7

  // 二阶平滑器实例
  SecondOrderSmoother angular_smoother_;

  // 控制计算的中间结果（用于数据记录）
  struct ControlComputationData
  {
    double deadzone_factor = 1.0;
    double d_adaptive_factor = 1.0;
    double pid_raw_output = 0.0;
    double angular_acceleration = 0.0;
    double effective_max_angular_vel = 0.0;
    double effective_max_angular_accel = 0.0;
  } control_data_;

  // ================================
  // IMU相关
  // ================================

  rclcpp::Node::SharedPtr imu_node_;  // IMU专用内部节点（职责单一，仅用于IMU订阅）
  std::thread imu_thread_;  // IMU数据处理独立线程
  std::atomic<bool> imu_thread_running_;  // IMU线程运行标志
  mutable std::mutex terrain_data_mutex_;  // 地形数据访问互斥锁
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;  // IMU订阅器
  
  // 运动状态检测相关
  rclcpp::Node::SharedPtr motion_node_;  // 运动状态检测专用节点
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;  // cmd_vel订阅器

  // ================================
  // ROS2实时日志系统
  // ================================
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr terrain_log_publisher_;  // 地形日志发布器
  std::string terrain_log_topic_name_;  // 地形日志话题名称

  // ================================
  // 私有方法
  // ================================

  // 参数管理
  void updateParameters();
  void initializeFilters();
  void resetControllerState();
  void initializeDefaultParameters();

  // 状态判断
  bool shouldGoBackward(double curr_x, double curr_y, double curr_yaw, double target_x, double target_y,
                        double target_yaw);
  bool isBeyondGoal(double robot_x, double robot_y);
  bool isAlignedWithTarget(double robot_yaw, const geometry_msgs::msg::Quaternion& target_orientation,
                           bool is_backward);

  // 路径管理
  geometry_msgs::msg::PoseStamped getNextWaypoint(double robot_x, double robot_y);
  size_t findNearestSegment(double robot_x, double robot_y);
  double computeCrossTrackError(double robot_x, double robot_y);
  void extendPath();  // 延长路径

  // 速度计算
  double computeLinearSpeed(double distance_to_target, double distance_to_start);
  double computeAngularVelocity(double yaw_error, double dt, double distance_to_target, double distance_to_start,
                                double linear_speed);
  double calculateRotationVelocity(const double& angle_diff);

  // 控制逻辑
  bool handleStateAlignment(double robot_yaw, const geometry_msgs::msg::Quaternion& target_orientation,
                            geometry_msgs::msg::TwistStamped& cmd_vel, bool is_backward = false);
  void handlePathFollowing(double robot_x, double robot_y, geometry_msgs::msg::TwistStamped& cmd_vel);
  bool handleWaitingState(geometry_msgs::msg::TwistStamped& cmd_vel);

  // 工具方法
  double getDeltaTime();
  double normalizeAngle(double angle);
  double distanceToSegment(double x, double y, double x1, double y1, double x2, double y2);
  void exportDebugData(const std::string& file_path, const std::vector<std::vector<double>>& data);

  // IMU相关方法
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void initializeImuSubscription();
  void spinImuNode();  // 处理IMU节点的消息（已弃用，保留兼容性）
  void imuThreadLoop();  // IMU独立线程循环函数

  // 三场景地形分析算法
  void updateImuHistory(const sensor_msgs::msg::Imu::SharedPtr msg);
  TerrainFeatures extractTerrainFeatures();
  TerrainType classifyTerrainWithPriority(const TerrainFeatures& features);
  void adaptControlParameters(TerrainType terrain_type, double confidence);
  
  // === 地形状态机核心方法 ===
  TerrainType manageTerrainImmediate(const TerrainFeatures& features);  // 立即切换地形状态
  void updateStateMemory(TerrainType current_terrain);  // 状态记忆更新
  
  
  // 线程安全的地形数据访问方法
  TerrainAnalysisData getTerrainDataSafe() const;  // 线程安全获取地形数据
  TerrainType getCurrentTerrainTypeSafe() const;  // 线程安全获取当前地形类型
  

  // 特征提取算法
  double computeMaxGradient();                              // 突变检测
  double computeZeroCrossingRate();                         // 频繁颠簸检测
  double computeLowFreqEnergyRatio();                       // 平缓凹凸检测
  double computeVariance(const std::vector<double>& data);  // 统计辅助函数

  // 参数读取辅助方法

  // 地形类型转换和辅助方法
  const TerrainControlParams& selectTerrainParams(TerrainType terrain_type) const;
  std::string terrainTypeToString(TerrainType type) const;

  // 数据记录方法
  void recordImuTerrainData();  // 记录IMU地形数据
  void exportImuTerrainData();  // 导出IMU地形数据
  std::string generateTaskId();

  // 控制效果评估方法
  double calculateControlStabilityScore(double current_angular_vel, double prev_angular_vel);
  double calculateTrackingAccuracyScore(double cross_track_error);
  double calculateResponseTimeScore(double yaw_error, double angular_response);

  // IMU统计计算方法
  void calculateRollStatistics(double& mean, double& std_dev, double& max_val, double& min_val, double& range);

  // ROS2实时日志系统方法
  void initializeTerrainLogPublisher();         // 初始化地形日志发布器
  void publishTerrainLogData(const TerrainFeatures& features, TerrainType classified_type, double confidence);  // 发布地形日志数据
  void publishControlLogData(double yaw_error, double cross_track_error, const geometry_msgs::msg::TwistStamped& cmd_vel,
                            bool back_follow, bool in_deadzone, double deadzone_factor, double d_adaptive_factor,
                            double max_angular_vel, double suppression_factor, double cross_track_deadzone,
                            double yaw_deadzone, double max_angular_accel, double current_angular_accel, 
                            bool imu_terrain_enabled);  // 发布控制日志数据
  std::string createTerrainJsonMessage(const TerrainFeatures& features, TerrainType classified_type, double confidence);  // 创建JSON消息
  std::string createControlJsonMessage(double yaw_error, double cross_track_error, const geometry_msgs::msg::TwistStamped& cmd_vel,
                                      bool back_follow, bool in_deadzone, double deadzone_factor, double d_adaptive_factor,
                                      double max_angular_vel, double suppression_factor, double cross_track_deadzone,
                                      double yaw_deadzone, double max_angular_accel, double current_angular_accel,
                                      bool imu_terrain_enabled);  // 创建控制日志JSON消息
};

}  // namespace follow_controller
}  // namespace xline
