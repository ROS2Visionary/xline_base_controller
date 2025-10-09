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

  // 三场景地形检测阈值参数（基于航空级IMU精度）


  // 地形类型枚举（按优先级排序）

  
  // 地形控制配置结构（立即切换模式）

    
  // 地形状态记录结构（简化版）

  

  // 地形特征数据结构（基于航空级IMU精度优化）


  // 地形分析数据结构（三场景版本）

  
  // 地形控制成员变量

  
  // 水泥地面模式相关变量


  // IMU历史数据存储（增强版 - 存储翻滚角和俯仰角用于突变检测）




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
  TerrainControlParams smooth_terrain_params_;     // 平稳地形参数（基线）
  TerrainControlParams alignment_params_;          // 通用对齐阶段参数

  // 翻滚角数据（用于调试和记录）


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


  // IMU地形数据记录结构（扩展版 - 支持三种场景）




  std::vector<std::vector<double>> original_path_;         // 原始路径点
  std::vector<std::vector<double>> filtered_path_;         // 滤波后路径点
  std::mutex file_mutex_;                                  // 文件操作互斥锁


  // 可配置的数据记录参数
  // std::string data_log_base_path_;                       // 数据日志基础路径（可配置）
  // bool enable_detailed_logging_;                         // 是否启用详细日志记录
  // bool enable_imu_terrain_logging_;                      // 是否启用IMU地形数据记录
  // double logging_frequency_;                             // 日志记录频率（Hz）
  // std::chrono::steady_clock::time_point last_log_time_;  // 上次记录时间

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


  // ================================
  // IMU相关
  // ================================


  
  // 运动状态检测相关


  // ================================
  // ROS2实时日志系统
  // ================================
  

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


  // 三场景地形分析算法

  
  // === 地形状态机核心方法 ===

  
  
  // 线程安全的地形数据访问方法

  

  // 特征提取算法


  // 参数读取辅助方法

  // 地形类型转换和辅助方法


  // 数据记录方法


  // 控制效果评估方法


  // IMU统计计算方法


  // ROS2实时日志系统方法

};

}  // namespace follow_controller
}  // namespace xline
