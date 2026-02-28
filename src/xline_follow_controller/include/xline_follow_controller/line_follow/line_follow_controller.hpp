#pragma once

#include "xline_follow_controller/common/base_follow_controller.hpp"
#include "xline_follow_controller/common/follow_common.hpp"
#include "xline_follow_controller/common/logging_compat.hpp"
#include "xline_follow_controller/line_follow/line_params.hpp"
#include "xline_follow_controller/lqr_follow/lqr_path_types.hpp"
#include "xline_follow_controller/common/yaml_parser.hpp"
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


/**
 * @brief 直线路径跟随控制器
 *
 * 主要用于处理“起点 -> 终点”的直线路径或简单折线：
 * - 从 YAML 文件加载一整套参数（速度、距离、阶段控制等），并映射到结构体 LineParams；
 * - 在起点、路径跟随、终点对齐几个阶段之间切换控制状态；
 * - 对线速度使用加速/减速 Sigmoid 曲线，尽量保证起步和刹车都比较平滑；
 * - 对角速度使用 PID + Hampel + Savitzky-Golay + 二阶平滑器做多层滤波；
 * - 支持后退模式、工作/非工作两种速度档以及调试路径导出。
 *
 * 该控制器实现了 BaseFollowController 的接口，供上层 MotionControlCenter 统一调度。
 */
class LineFollowController : public BaseFollowController
{
public:
  LineFollowController();
  ~LineFollowController();

  // 基类接口实现
  bool setPlan(const nav_msgs::msg::Path& orig_global_plan) override;
  bool computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose, 
                               const geometry_msgs::msg::Twist& velocity,
                               geometry_msgs::msg::TwistStamped& cmd_vel) override;
  bool isGoalReached() override;
  bool cancel() override;

  // 扩展接口
  /// 通过起点/终点坐标直接生成直线路径
  bool setPlan(double start_x, double start_y, double end_x, double end_y);
  /// 通过一组离散的 Pose 点设置路径（常用于上层自定义路径）
  bool setPlan(const std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped>>& plan);
  /// 限制最大运行速度（会同时调整内部运行时参数和阶段权重）
  void setSpeedLimit(const double& speed_limit);
  /// 切换是否采用后退方式跟随路径
  void setBackFollow(bool back);
  /// 标记是否为“转场路径”（用于缩短起步进入跟随阶段的条件）
  void setTransitionPath(bool is_transition);
  /// 外部更新当前机器人位姿（部分逻辑会用到缓存位姿）
  void setPose(const geometry_msgs::msg::PoseStamped& pose);

  /// 获取当前参数（只读）
  const LineParams& getParams() const { return params_; }

private:
  // ================================
  // 状态管理
  // ================================

  /**
   * @brief 控制器状态枚举
   */
  enum class ControlState
  {
    IDLE,            ///< 空闲状态
    ALIGNING_START,  ///< 对齐起始航向
    FOLLOWING_PATH,  ///< 跟随路径
    ALIGNING_END,    ///< 对齐终点航向
    GOAL_REACHED,    ///< 到达目标
    WAITING          ///< 等待状态
  };

  ControlState current_state_;        ///< 当前控制器状态
  bool received_plan_;                ///< 是否接收到路径
  bool goal_reached_;                 ///< 是否到达目标
  bool back_follow_;                  ///< 是否后退模式
  bool short_path_;                   ///< 是否为短路径
  bool decel_phase_entered_;          ///< 是否已进入减速阶段
  bool is_transition_path_;           ///< 是否为转场路径（外部设置）

  // 参数结构体（从YAML加载，只读）
  LineParams params_;                 ///< 控制器参数（结构体形式）


  // 速度限制（可动态修改）
  double runtime_walk_max_vel_;           ///< 步行最大速度 [m/s]
  double runtime_work_max_vel_;           ///< 工作最大速度 [m/s]
  double runtime_accel_min_vel_;          ///< 加速时最小线速度 [m/s]
  double runtime_decel_min_vel_;          ///< 减速时最小线速度 [m/s]
  double runtime_alignment_vel_;          ///< 对齐阶段速度 [m/s]

  // 距离参数（短路径时动态调整）
  double runtime_alignment_dist_;         ///< 对齐距离 [m]
  double runtime_decel_dist_;             ///< 减速距离 [m]
  double runtime_accel_dist_;             ///< 加速距离 [m]
  

  // 阶段运行时参数（可被setSpeedLimit修改权重）
  LinePhaseRuntimeParams following_params_;     ///< 跟随阶段参数
  LinePhaseRuntimeParams alignment_params_;     ///< 对齐阶段参数
  

  // 运行时状态变量
  double max_linear_speed_;           ///< 当前最大线速度（计算用）
  double current_linear_speed_;       ///< 当前线速度
  double current_angular_speed_;      ///< 当前角速度
  double robot_yaw_;                  ///< 机器人当前偏航角
  double alpha_;                      ///< 当前低通滤波系数
  bool start_line_aligned_;           ///< 起步阶段：是否已经满足“上直线+航向对齐”
  int start_line_aligned_count_;      ///< 连续满足计数（抗抖）
  double accel_start_distance_to_start_;  ///< 允许加速时的起点距离（用于加速曲线从0开始）

  // 路径和位姿信息
  nav_msgs::msg::Path global_plan_;                       ///< 全局路径
  geometry_msgs::msg::PoseStamped target_pose_;           ///< 目标位姿
  geometry_msgs::msg::PoseStamped end_pose_;              ///< 终点位姿
  geometry_msgs::msg::PoseStamped current_pose_;          ///< 当前位姿
  geometry_msgs::msg::PoseStamped original_target_pose_;  ///< 原始目标位姿（延长前）
  geometry_msgs::msg::PoseStamped original_start_pose_;  ///< 原始起点位姿

  size_t current_waypoint_index_;     ///< 当前路径点索引
  double path_length_;                ///< 路径长度
  double original_path_length_;       ///< 原始路径长度（延长前）

  // 路径延长参数
  // 为了保证终点对齐时有一定"缓冲区"，在原始终点之后再向前延长一小段直线。
  static constexpr double PATH_EXTENSION_LENGTH = 0.5;  ///< 路径延长距离 [m]
  static constexpr double PATH_POINT_INTERVAL = 0.003;  ///< 路径点间隔 [m]

  // LQR 相关成员变量
  std::vector<PathPointWithCurvature> path_with_curvature_;  ///< 带曲率的路径
  double K1_;                         ///< LQR 横向误差增益
  double K2_;                         ///< LQR 航向误差增益
  double current_min_K1_;
  size_t last_nearest_idx_;           ///< 上次最近点索引（用于优化搜索）
  double last_omega_;                 ///< 上次角速度（用于角加速度限制）
  SecondOrderSmoother lqr_angular_smoother_;  ///< LQR 输出二阶平滑器（独立于 PID）
  double prev_lqr_smoothed_omega_;    ///< LQR 输出低通记忆项（独立于 PID）
  double integral_lqr_e_y_;           ///< LQR 横向误差积分状态（可选）
  double prev_ey_rate_;               ///< 速率限制器记忆项（上帧限制后的 e_y）
  double prev_ey_lowpass_;            ///< 一阶低通记忆项

  std::shared_ptr<PIDController> heading_pid_controller_;  ///< 航向 PID 控制器

  // 位置滤波
  SavitzkyGolayFilter x_filter_;      ///< X坐标SG滤波器
  SavitzkyGolayFilter y_filter_;      ///< Y坐标SG滤波器
  HampelFilter h_x_filter_;           ///< X坐标Hampel滤波器
  HampelFilter h_y_filter_;           ///< Y坐标Hampel滤波器

  // 角速度滤波
  double prev_angular_velocity_;      ///< 上一次角速度
  double prev_smoothed_angular_velocity_; ///< 上一次平滑后角速度
  double last_yaw_error_;             ///< 上一次航向误差
  double second_prev_angular_velocity_;
  std::deque<double> angular_vel_history_; ///< 角速度历史记录
  HampelFilter angular_vel_hampel_filter_; ///< 角速度Hampel滤波器

  // 二阶平滑器
  SecondOrderSmoother angular_smoother_; ///< 二阶平滑器实例

  std::chrono::steady_clock::time_point last_time_;        ///< 上次计算时间
  std::chrono::steady_clock::time_point wait_start_time_;  ///< 等待开始时间
  double wait_duration_;              ///< 等待持续时间
  bool waiting_;                      ///< 是否在等待

  // ================================
  // 调试数据
  // ================================

  std::vector<std::vector<double>> original_path_;   ///< 原始路径点
  std::vector<std::vector<double>> filtered_path_;   ///< 滤波后路径点
  std::mutex file_mutex_;             ///< 文件操作互斥锁

  // 调试数据导出路径（解析后的完整路径）
  std::string resolved_debug_raw_dir_;
  std::string resolved_debug_filtered_dir_;

  // ================================
  // 私有方法 - 参数管理
  // ================================

  void updateParameters();
  void syncRuntimeParams();           ///< 将结构体参数同步到运行时变量
  void initializeFilters();
  void resetControllerState();

  // ================================
  // 私有方法 - 状态判断
  // ================================

  bool shouldGoBackward(double curr_x, double curr_y, double curr_yaw, 
                        double target_x, double target_y, double target_yaw);
  bool isBeyondGoal(double robot_x, double robot_y);
  bool isAlignedWithTarget(double robot_yaw, 
                           const geometry_msgs::msg::Quaternion& target_orientation,
                           bool is_backward);
  void updateStartLineAlignment(double robot_x, double robot_y, double robot_yaw, double distance_to_start);

  // ================================
  // 私有方法 - 路径管理
  // ================================

  geometry_msgs::msg::PoseStamped getNextWaypoint(double robot_x, double robot_y);
  size_t findNearestSegment(double robot_x, double robot_y);
  double computeCrossTrackError(double robot_x, double robot_y);
  void extendPath();
  double getLookaheadDistance(double speed);

  // ================================
  // 私有方法 - 速度计算
  // ================================

  double computeLinearSpeed(double distance_to_target, double distance_to_start);
  double computeAngularVelocity(double yaw_error, double dt,
                                double distance_to_target, double distance_to_start,
                                double linear_speed);
  double calculateRotationVelocity(const double& angle_diff);

  // ================================
  // 私有方法 - LQR 控制
  // ================================

  void computeLQRGains(double v);
  void computeLQRErrors(double current_x, double current_y, double current_theta,
                        const PathPointWithCurvature& ref,
                        double& e_y, double& e_theta);
  double computeAngularVelocityLQR(double robot_x, double robot_y, double robot_yaw,
                                   double linear_speed, double dt);
  size_t findNearestPointIndex(double x, double y);
  size_t findLookaheadPointIndex(size_t start_idx, double robot_x, double robot_y,
                                 double lookahead_dist);
  double applyAngularLimits(double omega, double dt);

  // ================================
  // 私有方法 - 控制逻辑
  // ================================

  bool handleStateAlignment(double robot_yaw, 
                            const geometry_msgs::msg::Quaternion& target_orientation,
                            geometry_msgs::msg::TwistStamped& cmd_vel, 
                            bool is_backward = false);
  void handlePathFollowing(double robot_x, double robot_y, 
                           geometry_msgs::msg::TwistStamped& cmd_vel);
  bool handleWaitingState(geometry_msgs::msg::TwistStamped& cmd_vel);

  // ================================
  // 私有方法 - 工具方法
  // ================================

  double getDeltaTime();
  double normalizeAngle(double angle);
  double distanceToSegment(double x, double y, double x1, double y1, double x2, double y2);
  void exportDebugData(const std::string& file_path, 
                       const std::vector<std::vector<double>>& data);

  // ================================
  // 便捷访问器（简化代码中对params_的访问）
  // ================================

  // 调试
  inline bool isDebugEnabled() const { return params_.debug.enabled; }
  
  // PID参数
  inline const LinePIDGains& alignmentPID() const { return params_.heading_pid.alignment; }
  inline const LinePIDGains& followingPID() const { return params_.heading_pid.following; }
  
  // 旋转参数
  inline double rotationMaxW() const { return params_.rotation.max_w; }
  inline double rotationMinW() const { return params_.rotation.min_w; }
  inline double rotationFactor() const { return params_.rotation.factor; }
  inline double rotationAngleThreshold() const { return params_.rotation.angle_threshold; }
  inline double rotationSmoothFactor() const { return params_.rotation.smooth_factor; }
  
  // 距离参数
  inline double lookaheadDist() const { return params_.distance.lookahead; }
  inline double waypointTolerance() const { return params_.distance.waypoint_tolerance; }

  
  // 速度调节参数
  inline double acceFactor() const { return params_.velocity.acce_factor; }
  inline double accelSigmoidK() const { return params_.velocity.accel_sigmoid_k; }
  inline double decelSigmoidK() const { return params_.velocity.decel_sigmoid_k; }
  inline double accelSigmoidCenter() const { return params_.velocity.accel_sigmoid_center; }
  inline double decelSigmoidCenter() const { return params_.velocity.decel_sigmoid_center; }
};

}  // namespace follow_controller
}  // namespace xline
