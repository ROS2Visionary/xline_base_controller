#pragma once

#include "follow_controller/base_follow_controller.hpp"
#include "follow_controller/follow_common.hpp"
#include <yaml-cpp/yaml.h>
#include "daosnrs_common/logger.h"
#include "daosnrs_common/geometry/base_path.hpp"
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
#include <cmath>
#include <algorithm>
#include <std_msgs/msg/string.hpp>

namespace xline
{
namespace follow_controller
{

class LineFollowController : public BaseFollowController
{
public:
  LineFollowController();
  ~LineFollowController();

  // =========================== 基础接口函数 ===========================
  bool setPlan(const nav_msgs::msg::Path& orig_global_plan) override;
  bool computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
                               geometry_msgs::msg::TwistStamped& cmd_vel) override;
  bool isGoalReached() override;
  bool cancel() override;

  // =========================== 扩展接口函数 ===========================
  bool setPlanForBasePath(const std::shared_ptr<daosnrs::geometry::BasePath>& path_object);
  bool setPlan(const std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped>>& plan);
  void setSpeedLimit(const double& speed_limit);
  void setWorkState(bool state);
  void setBackFollow(bool back);
  void setPose(const geometry_msgs::msg::PoseStamped& pose);

  void startImuThread();

  // =========================== 地形控制参数结构 ===========================
  struct TerrainControlParams
  {
    double cross_track_deadzone;  // 横向偏差死区
    double yaw_deadzone;          // 航向偏差死区
    double max_angular_vel;       // 最大角速度
    double max_angular_accel;     // 最大角加速度
    double suppression_factor;    // 抑制因子

    double current_heading_weight;  // 当前航向权重
    double target_heading_weight;   // 目标航向权重

    double alpha;  // 平滑参数

    double smoother_frequency;  // 平滑器频率
    double smoother_damping;    // 平滑器阻尼
  };

private:
  // =========================== 控制状态枚举 ===========================
  enum class ControlState
  {
    IDLE,            // 空闲状态
    ALIGNING_START,  // 起点对齐状态
    FOLLOWING_PATH,  // 路径跟随状态
    ALIGNING_END,    // 终点对齐状态
    GOAL_REACHED,    // 目标到达状态
    WAITING          // 等待状态
  };

  // =========================== 状态变量 ===========================
  ControlState current_state_;  // 当前控制状态
  bool received_plan_;          // 是否收到路径规划
  bool goal_reached_;           // 是否到达目标
  bool back_follow_;            // 是否后退跟随
  bool m_work_state_;           // 工作状态
  bool short_path_;             // 是否为短路径
  bool reset_required_;         // 是否需要重置
  bool debug_;                  // 调试模式
  bool decel_phase_entered_;    // 是否进入减速阶段

  // =========================== 路径和位姿信息 ===========================
  nav_msgs::msg::Path global_plan_;                       // 全局路径规划
  geometry_msgs::msg::PoseStamped start_pose_;            // 起始位姿
  geometry_msgs::msg::PoseStamped target_pose_;           // 目标位姿
  geometry_msgs::msg::PoseStamped end_pose_;              // 结束位姿
  geometry_msgs::msg::PoseStamped current_pose_;          // 当前位姿
  geometry_msgs::msg::PoseStamped original_target_pose_;  // 原始目标位姿

  size_t current_waypoint_index_;  // 当前路径点索引
  double path_length_;             // 路径长度
  double original_path_length_;    // 原始路径长度
  double robot_yaw_;               // 机器人航向角

  // =========================== 路径扩展常量 ===========================
  static constexpr double PATH_EXTENSION_LENGTH = 0.5;  // 路径延长长度(米)
  static constexpr double PATH_POINT_INTERVAL = 0.001;  // 路径点间隔(米)

  // =========================== 线速度控制参数 ===========================
  double max_linear_speed_;      // 最大线速度
  double min_linear_speed_;      // 最小线速度
  double current_linear_speed_;  // 当前线速度
  double m_walk_max_vel_;        // 行走最大速度
  double m_work_max_vel_;        // 工作最大速度
  double m_alignment_vel_;       // 对齐速度
  double mini_path_distance_;    // 最小路径距离

  // =========================== 角速度控制参数 ===========================
  double max_rotation_angular_vel_;      // 旋转最大角速度
  double min_rotation_angular_vel_;      // 旋转最小角速度
  double current_angular_speed_;         // 当前角速度
  double rotation_angular_factor_;       // 旋转角速度因子
  double rotation_angle_threshold_;      // 旋转角度阈值
  double rotation_angle_smooth_factor_;  // 旋转角度平滑因子

  // =========================== IMU地形自适应参数 ===========================
  bool enable_imu_terrain_adaptation_;  // 是否启用IMU地形自适应
  double imu_window_size_;              // IMU数据窗口大小

  double sudden_change_gradient_threshold_;  // 突变梯度阈值
  double gentle_low_freq_energy_threshold_;  // 缓慢起伏低频能量阈值
  double frequent_zero_crossing_threshold_;  // 频繁颠簸过零率阈值

  // =========================== 地形类型枚举 ===========================
  enum class TerrainType
  {
    SMOOTH = 0,             // 平滑地形
    SUDDEN_CHANGE = 1,      // 突变地形
    GENTLE_UNDULATION = 2,  // 缓慢起伏
    FREQUENT_BUMPS = 3,     // 频繁颠簸
    TRANSITIONING = 4,      // 过渡状态
    UNKNOWN = 5             // 未知地形
  };

  // =========================== 地形控制配置 ===========================
  struct TerrainControlConfig
  {
    bool immediate_switch = true;  // 是否立即切换地形参数
  };

  // =========================== 地形状态记忆 ===========================
  struct TerrainStateMemory
  {
    TerrainType previous_terrain = TerrainType::SMOOTH;              // 前一次地形类型
    std::chrono::steady_clock::time_point last_terrain_change_time;  // 上次地形变化时间
    int terrain_change_count = 0;                                    // 地形变化次数
    int sudden_change_count = 0;                                     // 突变次数
  };

  // =========================== 地形特征结构 ===========================
  struct TerrainFeatures
  {
    double max_gradient = 0.0;                          // 最大梯度
    double zero_crossing_rate = 0.0;                    // 过零率
    double low_freq_energy_ratio = 0.0;                 // 低频能量比
    std::chrono::steady_clock::time_point last_update;  // 上次更新时间
  };

  // =========================== 地形分析数据 ===========================
  struct TerrainAnalysisData
  {
    TerrainType current_type = TerrainType::SMOOTH;     // 当前地形类型
    TerrainFeatures features;                           // 地形特征
    double confidence_score = 0.0;                      // 置信度分数
    double angle_vel_dynamic_factor = 1.0;              // 角速度动态因子
    double line_vel_dynamic_factor = 1.0;               // 线速度动态因子
    std::chrono::steady_clock::time_point last_update;  // 上次更新时间
  };

  TerrainAnalysisData terrain_data_;  // 地形分析数据

  // =========================== 地形控制实例 ===========================
  TerrainControlConfig terrain_config_;  // 地形控制配置
  TerrainStateMemory state_memory_;      // 地形状态记忆

  // =========================== 当前IMU数据 ===========================
  double current_pitch_angle_ = 0.0;  // 当前俏仰角
  double current_yaw_rate_ = 0.0;     // 当前航向角速度

  // =========================== IMU数据点结构 ===========================
  struct ImuDataPoint
  {
    double roll;                                      // 翻滚角
    double pitch;                                     // 俏仰角
    std::chrono::steady_clock::time_point timestamp;  // 时间戳
  };

  std::deque<ImuDataPoint> imu_history_;  // IMU历史数据队列
  size_t max_imu_history_size_;           // IMU历史数据最大大小

  // =========================== 自适应控制参数 ===========================
  struct AdaptiveControlParams
  {
    // 当前自适应参数
    double current_cross_track_deadzone = 0.006;  // 当前横向偏差死区
    double current_yaw_deadzone = 0.02;           // 当前航向偏差死区
    double current_max_angular_vel = 0.05;        // 当前最大角速度
    double current_max_angular_accel = 0.1;       // 当前最大角加速度
    double current_suppression_factor = 0.5;      // 当前抑制因子

    double current_heading_weight = 0.0;  // 当前航向权重
    double target_heading_weight = 1.0;   // 目标航向权重

    bool high_precision_active = true;  // 高精度模式激活
  };

  AdaptiveControlParams adaptive_params_;  // 自适应控制参数实例

  // =========================== 不同地形的控制参数 ===========================
  TerrainControlParams sudden_change_params_;      // 突变地形参数
  TerrainControlParams gentle_undulation_params_;  // 缓慢起伏参数
  TerrainControlParams frequent_bumps_params_;     // 频繁颠簸参数
  TerrainControlParams smooth_terrain_params_;     // 平滑地形参数
  TerrainControlParams alignment_params_;          // 对齐参数

  // =========================== 翻滚角数据 ===========================
  double current_roll_angle_;   // 当前翻滚角
  double filtered_roll_angle_;  // 过滤后翻滚角

  // =========================== 路径跟随公差参数 ===========================
  double waypoint_tolerance_;     // 路径点容差
  double yaw_tolerance_;          // 航向角容差
  double lookahead_distance_;     // 前瞻距离
  double deceleration_distance_;  // 减速距离
  double m_alignment_distance_;   // 对齐距离
  double m_acce_distance_;        // 加速距离

  // =========================== 加速度控制参数 ===========================
  double m_acce_factor_;                  // 加速因子
  double m_acceleration_factor_;          // 加速系数
  double m_deceleration_factor_;          // 减速系数
  double m_acceleration_sigmoid_center_;  // 加速Sigmoid中心
  double m_deceleration_sigmoid_center_;  // 减速Sigmoid中心
  double max_angular_acceleration_;       // 最大角加速度

  // =========================== PID控制器 ===========================
  std::shared_ptr<PIDController> pid_heading_controller_;  // 航向PID控制器
  double angular_kp_;                                      // PID比例系数
  double angular_ki_;                                      // PID积分系数
  double angular_kd_;                                      // PID微分系数

  // =========================== 滤波器 ===========================
  SavitzkyGolayFilter x_filter_;  // X坐标Savitzky-Golay滤波器
  SavitzkyGolayFilter y_filter_;  // Y坐标Savitzky-Golay滤波器
  HampelFilter h_x_filter_;       // X坐标Hampel滤波器
  HampelFilter h_y_filter_;       // Y坐标Hampel滤波器

  int m_window_size_;       // 滤波窗口大小
  int m_polynomial_order_;  // 多项式阶数
  int m_hampel_window_;     // Hampel滤波窗口
  double m_hampel_k_;       // Hampel滤波系数

  // =========================== 角速度平滑参数 ===========================
  double prev_angular_velocity_;            // 前一次角速度
  double prev_smoothed_angular_velocity_;   // 前一次平滑角速度
  double last_yaw_error_;                   // 上次航向偏差
  double second_prev_angular_velocity_;     // 前二次角速度
  double alpha_;                            // 平滑系数
  std::deque<double> angular_vel_history_;  // 角速度历史数据
  int angular_vel_history_size_ = 5;        // 角速度历史大小
  HampelFilter angular_vel_hampel_filter_;  // 角速度Hampel滤波器
  int angular_following_hampel_window_;     // 跟随Hampel窗口
  double angular_following_hampel_k_;       // 跟随Hampel系数

  // =========================== 当前平滑参数 ===========================
  double current_alpha_;               // 当前平滑系数
  double current_smoother_frequency_;  // 当前平滑器频率
  double current_smoother_damping_;    // 当前平滑器阻尼

  // =========================== 虚拟位置跟踪器 ===========================
  /**
   * @brief 虚拟位置跟踪器类
   *
   * 该类用于跟踪机器人的虚拟位置，结合里程计和实际测量位置
   * 进行自适应校正，提高在复杂地形下的位置精度。
   */
  class VirtualPositionTracker
  {
  public:
    VirtualPositionTracker()
      : virtual_x_(0.0)
      , virtual_y_(0.0)
      , virtual_yaw_(0.0)
      , initialized_(false)
      , last_update_time_(std::chrono::steady_clock::now())
      , correction_gain_(0.15)
      , max_correction_(0.005)
      , position_trust_threshold_(0.02)
      , max_velocity_trust_(0.5)
      , angular_velocity_factor_(1.0)
      , smooth_gain_(0.25)
      , rough_gain_(0.08)
      , sudden_change_gain_(0.05)
      , debug_logging_(false)
      , log_frequency_(50)
      , log_counter_(0)
    {
      // 初始化虚拟位置跟踪器
    }

    // 初始化虚拟位置
    void initialize(double x, double y, double yaw)
    {
      virtual_x_ = x;
      virtual_y_ = y;
      virtual_yaw_ = yaw;
      initialized_ = true;
      last_update_time_ = std::chrono::steady_clock::now();
    }

    // 更新虚拟位姿（基于里程计）
    void updateVirtualPose(double linear_vel, double angular_vel, double dt)
    {
      if (!initialized_)
        return;

      if (std::abs(linear_vel) > max_velocity_trust_)
      {
        linear_vel = std::copysign(max_velocity_trust_, linear_vel);
      }

      double dx = linear_vel * std::cos(virtual_yaw_) * dt;
      double dy = linear_vel * std::sin(virtual_yaw_) * dt;

      virtual_x_ += dx;
      virtual_y_ += dy;
      virtual_yaw_ += angular_vel * angular_velocity_factor_ * dt;

      while (virtual_yaw_ > M_PI)
        virtual_yaw_ -= 2.0 * M_PI;
      while (virtual_yaw_ < -M_PI)
        virtual_yaw_ += 2.0 * M_PI;

      last_update_time_ = std::chrono::steady_clock::now();
    }

    // 使用实际测量数据校正虚拟位置
    void correctWithMeasurement(double measured_x, double measured_y, TerrainType terrain_type)
    {
      if (!initialized_)
      {
        initialize(measured_x, measured_y, 0.0);
        return;
      }

      double error_x = measured_x - virtual_x_;
      double error_y = measured_y - virtual_y_;
      double position_error = std::sqrt(error_x * error_x + error_y * error_y);

      double adaptive_gain = getAdaptiveGain(terrain_type);

      if (position_error > position_trust_threshold_)
      {
        adaptive_gain *= 0.3;
      }

      double correction_x = std::clamp(error_x, -max_correction_, max_correction_);
      double correction_y = std::clamp(error_y, -max_correction_, max_correction_);

      virtual_x_ += adaptive_gain * correction_x;
      virtual_y_ += adaptive_gain * correction_y;

      if (debug_logging_ && (++log_counter_ % log_frequency_ == 0))
      {
        logTrackingData(measured_x, measured_y, error_x, error_y, adaptive_gain, terrain_type);
      }
    }

    // 获取虚拟位置信息
    double getVirtualX() const
    {
      return virtual_x_;
    }
    double getVirtualY() const
    {
      return virtual_y_;
    }
    double getVirtualYaw() const
    {
      return virtual_yaw_;
    }
    bool isInitialized() const
    {
      return initialized_;
    }

    // 设置跟踪器参数
    void setParameters(double correction_gain, double max_correction, double position_trust_threshold,
                       double max_velocity_trust, double angular_velocity_factor)
    {
      correction_gain_ = correction_gain;
      max_correction_ = max_correction;
      position_trust_threshold_ = position_trust_threshold;
      max_velocity_trust_ = max_velocity_trust;
      angular_velocity_factor_ = angular_velocity_factor;
    }

    // 设置不同地形的校正增益
    void setTerrainGains(double smooth_gain, double rough_gain, double sudden_change_gain)
    {
      smooth_gain_ = smooth_gain;
      rough_gain_ = rough_gain;
      sudden_change_gain_ = sudden_change_gain;
    }

    // 设置调试日志
    void setDebugLogging(bool enabled, int frequency)
    {
      debug_logging_ = enabled;
      log_frequency_ = frequency;
    }

    // 重置跟踪器
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
    // =========================== 虚拟位置状态 ===========================
    double virtual_x_, virtual_y_, virtual_yaw_;              // 虚拟位置坐标和航向
    bool initialized_;                                        // 是否已初始化
    std::chrono::steady_clock::time_point last_update_time_;  // 上次更新时间

    // =========================== 校正参数 ===========================
    double correction_gain_;           // 校正增益
    double max_correction_;            // 最大校正量
    double position_trust_threshold_;  // 位置信任阈值
    double max_velocity_trust_;        // 最大速度信任值
    double angular_velocity_factor_;   // 角速度因子

    // =========================== 地形相关增益 ===========================
    double smooth_gain_;         // 平滑地形增益
    double rough_gain_;          // 粗糙地形增益
    double sudden_change_gain_;  // 突变地形增益

    // =========================== 调试日志 ===========================
    bool debug_logging_;  // 是否启用调试日志
    int log_frequency_;   // 日志频率
    int log_counter_;     // 日志计数器

    // 根据地形类型获取自适应增益
    double getAdaptiveGain(TerrainType terrain_type)
    {
      switch (terrain_type)
      {
        case TerrainType::SMOOTH:
          return smooth_gain_;
        case TerrainType::GENTLE_UNDULATION:
        case TerrainType::FREQUENT_BUMPS:
          return rough_gain_;
        case TerrainType::SUDDEN_CHANGE:
          return sudden_change_gain_;
        default:
          return correction_gain_;
      }
    }

    // 记录跟踪数据
    void logTrackingData(double measured_x, double measured_y, double error_x, double error_y, double gain,
                         TerrainType terrain_type)
    {
    }
  };

  VirtualPositionTracker virtual_tracker_;  // 虚拟位置跟踪器实例

  // =========================== 虚拟跟踪配置 ===========================
  struct VirtualTrackingConfig
  {
    bool enabled = true;                     // 是否启用虚拟跟踪
    double correction_gain = 0.15;           // 校正增益
    double max_correction = 0.005;           // 最大校正量
    double position_trust_threshold = 0.02;  // 位置信任阈值
    double max_velocity_trust = 0.5;         // 最大速度信任值
    double angular_velocity_factor = 1.0;    // 角速度因子
    double smooth_gain = 0.25;               // 平滑地形增益
    double rough_gain = 0.08;                // 粗糙地形增益
    double sudden_change_gain = 0.05;        // 突变地形增益
    bool debug_logging = false;              // 调试日志
    int log_frequency = 50;                  // 日志频率
  } virtual_tracking_config_;

  // =========================== 时间控制 ===========================
  std::chrono::steady_clock::time_point last_time_;        // 上次更新时间
  std::chrono::steady_clock::time_point wait_start_time_;  // 等待开始时间
  double wait_duration_;                                   // 等待持续时间
  bool waiting_;                                           // 是否处于等待状态

  // =========================== 调试信息结构 ===========================
  struct DebugInfo
  {
    double target_speed;                              // 目标速度
    double distance_to_target;                        // 到目标距离
    double distance_to_start;                         // 到起点距离
    double cross_track_error;                         // 横向偏差
    double yaw_error;                                 // 航向偏差
    double yaw_pid_output;                            // 航向PID输出
    double filtered_pos_x;                            // 过滤后的X位置
    double filtered_pos_y;                            // 过滤后的Y位置
    std::chrono::steady_clock::time_point timestamp;  // 时间戳
  };

  // =========================== IMU地形数据结构 ===========================
  struct ImuTerrainData
  {
    double timestamp;      // 时间戳
    double current_roll;   // 当前翻滚角
    double filtered_roll;  // 过滤后翻滚角

    // 地形特征参数
    double max_gradient;           // 最大梯度
    double zero_crossing_rate;     // 过零率
    double low_freq_energy_ratio;  // 低频能量比
    std::string terrain_type;      // 地形类型
    double confidence_score;       // 置信度分数

    double dynamic_factor;  // 动态因子

    // 翻滚角统计信息
    size_t imu_history_size;  // IMU历史数据大小
    double roll_mean;         // 翻滚角平均值
    double roll_std_dev;      // 翻滚角标准差
    double roll_max;          // 翻滚角最大值
    double roll_min;          // 翻滚角最小值
    double roll_range;        // 翻滚角范围

    // 自适应控制参数
    double adaptive_cross_track_deadzone;  // 自适应横向偏差死区
    double adaptive_yaw_deadzone;          // 自适应航向偏差死区
    double adaptive_max_angular_vel;       // 自适应最大角速度
    double adaptive_max_angular_accel;     // 自适应最大角加速度
    double adaptive_suppression_factor;    // 自适应抑制因子
    bool high_precision_active;            // 高精度模式激活
  };

  // =========================== 数据存储 ===========================
  std::vector<std::vector<double>> original_path_;         // 原始路径数据
  std::vector<std::vector<double>> filtered_path_;         // 过滤后路径数据
  std::vector<ImuTerrainData> imu_terrain_data_;           // IMU地形数据列表
  std::mutex file_mutex_;                                  // 文件操作互斥锁
  std::string current_task_id_;                            // 当前任务ID
  std::chrono::steady_clock::time_point task_start_time_;  // 任务开始时间

  // =========================== 日志配置 ===========================
  std::string data_log_base_path_;                       // 数据日志基本路径
  bool enable_detailed_logging_;                         // 是否启用详细日志
  bool enable_imu_terrain_logging_;                      // 是否启用IMU地形日志
  double logging_frequency_;                             // 日志频率
  std::chrono::steady_clock::time_point last_log_time_;  // 上次日志时间

  // =========================== 角速度平滑器 ===========================
  double angular_smoother_freq_;     // 角速度平滑器频率
  double angular_smoother_damping_;  // 角速度平滑器阻尼

  SecondOrderSmoother angular_smoother_;  // 二阶平滑器

  // =========================== 控制计算数据 ===========================
  struct ControlComputationData
  {
    double deadzone_factor = 1.0;              // 死区因子
    double d_adaptive_factor = 1.0;            // 自适应因子
    double pid_raw_output = 0.0;               // PID原始输出
    double angular_acceleration = 0.0;         // 角加速度
    double effective_max_angular_vel = 0.0;    // 有效最大角速度
    double effective_max_angular_accel = 0.0;  // 有效最大角加速度
  } control_data_;

  // =========================== IMU线程和订阅 ===========================
  rclcpp::Node::SharedPtr imu_node_;                                         // IMU独立节点
  std::thread imu_thread_;                                                   // IMU处理线程
  std::atomic<bool> imu_thread_running_;                                     // IMU线程运行标志
  mutable std::mutex terrain_data_mutex_;                                    // 地形数据互斥锁
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;  // IMU数据订阅

  // =========================== 运动控制节点 ===========================
  rclcpp::Node::SharedPtr motion_node_;                                              // 运动控制节点
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;  // 速度命令订阅

  // =========================== 地形日志发布 ===========================
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr terrain_log_publisher_;  // 地形日志发布器
  std::string terrain_log_topic_name_;                                         // 地形日志话题名称

  // =========================== 初始化方法 ===========================
  void updateParameters();             // 更新参数
  void initializeFilters();            // 初始化滤波器
  void resetControllerState();         // 重置控制器状态
  void initializeDefaultParameters();  // 初始化默认参数

  // =========================== 状态判断方法 ===========================
  bool shouldGoBackward(double curr_x, double curr_y, double curr_yaw, double target_x, double target_y,
                        double target_yaw);           // 判断是否应该后退
  bool isBeyondGoal(double robot_x, double robot_y);  // 判断是否超过目标
  bool isAlignedWithTarget(double robot_yaw, const geometry_msgs::msg::Quaternion& target_orientation,
                           bool is_backward);  // 判断是否与目标对齐

  // =========================== 路径处理方法 ===========================
  geometry_msgs::msg::PoseStamped getNextWaypoint(double robot_x, double robot_y);  // 获取下一个路径点
  size_t findNearestSegment(double robot_x, double robot_y);                        // 找到最近线段
  double computeCrossTrackError(double robot_x, double robot_y);                    // 计算横向偏差
  void extendPath();                                                                // 扩展路径

  // =========================== 速度计算方法 ===========================
  double computeLinearSpeed(double distance_to_target, double distance_to_start);  // 计算线速度
  double computeAngularVelocity(double yaw_error, double dt, double distance_to_target, double distance_to_start,
                                double linear_speed);          // 计算角速度
  double calculateRotationVelocity(const double& angle_diff);  // 计算旋转速度

  // =========================== 状态处理方法 ===========================
  bool handleStateAlignment(double robot_yaw, const geometry_msgs::msg::Quaternion& target_orientation,
                            geometry_msgs::msg::TwistStamped& cmd_vel, bool is_backward = false);  // 处理对齐状态
  void handlePathFollowing(double robot_x, double robot_y, geometry_msgs::msg::TwistStamped& cmd_vel);  // 处理路径跟随
  bool handleWaitingState(geometry_msgs::msg::TwistStamped& cmd_vel);  // 处理等待状态

  // =========================== 工具方法 ===========================
  double getDeltaTime();                                                                     // 获取时间间隔
  double normalizeAngle(double angle);                                                       // 角度归一化
  double distanceToSegment(double x, double y, double x1, double y1, double x2, double y2);  // 到线段距离
  void exportDebugData(const std::string& file_path, const std::vector<std::vector<double>>& data);  // 导出调试数据

  // =========================== IMU相关方法 ===========================
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);  // IMU数据回调
  void initializeImuSubscription();                              // 初始化IMU订阅
  void spinImuNode();                                            // 运行IMU节点
  void imuThreadLoop();                                          // IMU线程循环

  // =========================== 地形分析方法 ===========================
  void updateImuHistory(const sensor_msgs::msg::Imu::SharedPtr msg);         // 更新IMU历史数据
  TerrainFeatures extractTerrainFeatures();                                  // 提取地形特征
  TerrainType classifyTerrainWithPriority(const TerrainFeatures& features);  // 地形分类
  void adaptControlParameters(TerrainType terrain_type, double confidence);  // 自适应控制参数

  TerrainType manageTerrainImmediate(const TerrainFeatures& features);  // 立即地形管理
  void updateStateMemory(TerrainType current_terrain);                  // 更新状态记忆

  // =========================== 线程安全方法 ===========================
  TerrainAnalysisData getTerrainDataSafe() const;  // 安全获取地形数据
  TerrainType getCurrentTerrainTypeSafe() const;   // 安全获取当前地形类型

  // =========================== 特征计算方法 ===========================
  double computeMaxGradient();                              // 计算最大梯度
  double computeZeroCrossingRate();                         // 计算过零率
  double computeLowFreqEnergyRatio();                       // 计算低频能量比
  double computeVariance(const std::vector<double>& data);  // 计算方差

  // =========================== 参数选择方法 ===========================
  const TerrainControlParams& selectTerrainParams(TerrainType terrain_type) const;  // 选择地形参数
  std::string terrainTypeToString(TerrainType type) const;                          // 地形类型转字符串

  // =========================== 数据记录方法 ===========================
  void recordImuTerrainData();   // 记录IMU地形数据
  void exportImuTerrainData();   // 导出IMU地形数据
  std::string generateTaskId();  // 生成任务ID

  // =========================== 性能评估方法 ===========================
  double calculateControlStabilityScore(double current_angular_vel, double prev_angular_vel);  // 计算控制稳定性分数
  double calculateTrackingAccuracyScore(double cross_track_error);               // 计算跟踪精度分数
  double calculateResponseTimeScore(double yaw_error, double angular_response);  // 计算响应时间分数

  // =========================== 统计计算方法 ===========================
  void calculateRollStatistics(double& mean, double& std_dev, double& max_val, double& min_val,
                               double& range);  // 计算翻滚角统计

  // =========================== 日志发布方法 ===========================
  void initializeTerrainLogPublisher();  // 初始化地形日志发布器
  void publishTerrainLogData(const TerrainFeatures& features, TerrainType classified_type,
                             double confidence);  // 发布地形日志数据
  void publishControlLogData(double yaw_error, double cross_track_error,
                             const geometry_msgs::msg::TwistStamped& cmd_vel, bool back_follow, bool in_deadzone,
                             double deadzone_factor, double d_adaptive_factor, double max_angular_vel,
                             double suppression_factor, double cross_track_deadzone, double yaw_deadzone,
                             double max_angular_accel, double current_angular_accel,
                             bool imu_terrain_enabled);  // 发布控制日志数据
  // =========================== JSON消息创建方法 ===========================
  std::string createTerrainJsonMessage(const TerrainFeatures& features, TerrainType classified_type,
                                       double confidence);  // 创建地形JSON消息
  std::string createControlJsonMessage(double yaw_error, double cross_track_error,
                                       const geometry_msgs::msg::TwistStamped& cmd_vel, bool back_follow,
                                       bool in_deadzone, double deadzone_factor, double d_adaptive_factor,
                                       double max_angular_vel, double suppression_factor, double cross_track_deadzone,
                                       double yaw_deadzone, double max_angular_accel, double current_angular_accel,
                                       bool imu_terrain_enabled);  // 创建控制JSON消息
};

}  // namespace follow_controller
}  // namespace xline