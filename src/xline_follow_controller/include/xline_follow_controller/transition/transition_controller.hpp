#pragma once

#include "xline_follow_controller/common/base_follow_controller.hpp"
#include "xline_follow_controller/common/logging_compat.hpp"
#include "xline_follow_controller/common/yaml_parser.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>

#include <chrono>
#include <cmath>
#include <algorithm>

namespace xline
{
namespace follow_controller
{

/**
 * @brief 动态伺服转场控制器
 *
 * 特点：
 * - 无需路径规划，直接反应式控制
 * - 边走边转，无需原地旋转
 * - 支持两种模式：只到点 / 到点+朝向
 * - 精度可控（通过配置调整）
 * - 计算高效（<0.1ms）
 *
 * 原理：
 * - 模式1（只到点）：
 *   v = k_p * d, ω = k_θ * (atan2(Δy, Δx) - θ_current)
 *
 * - 模式2（姿态伺服）：
 *   远距离：优先接近，航向指向目标点
 *   近距离：同时收敛位置和朝向
 *   v = k_p * d * cos(θ_error), ω = k_θ * θ_error
 *
 * 适用场景：
 * - 短距离转场（<3m）
 * - 无障碍环境
 * - 快速响应需求
 */
class TransitionController : public BaseFollowController
{
public:
  /**
   * @brief 构造函数
   */
  TransitionController();

  /**
   * @brief 析构函数
   */
  ~TransitionController() override;

  /**
   * @brief 设置目标点（只到点，不控制朝向）
   * @param goal_x 目标X坐标 [m]
   * @param goal_y 目标Y坐标 [m]
   * @return 成功返回true
   */
  bool setGoal(double goal_x, double goal_y);

  /**
   * @brief 设置目标姿态（到点+朝向，推荐）
   * @param goal_x 目标X坐标 [m]
   * @param goal_y 目标Y坐标 [m]
   * @param goal_theta 目标朝向 [rad]
   * @return 成功返回true
   */
  bool setGoal(double goal_x, double goal_y, double goal_theta);

  /**
   * @brief 设置路径（兼容接口，提取终点）
   */
  bool setPlan(const nav_msgs::msg::Path& path) override;

  /**
   * @brief 计算速度指令（核心函数）
   */
  bool computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose,
      const geometry_msgs::msg::Twist& velocity,
      geometry_msgs::msg::TwistStamped& cmd_vel) override;

  /**
   * @brief 是否到达目标
   */
  bool isGoalReached() override;

  /**
   * @brief 取消当前任务
   */
  bool cancel() override;

  /**
   * @brief 设置速度限制
   */
  void setSpeedLimit(const double& speed_limit) override;

  /**
   * @brief 重置控制器状态
   */
  void reset();

private:
  // ================================
  // 参数结构体
  // ================================

  /**
   * @brief 控制器参数
   */
  struct Parameters {
    // 速度限制
    double max_velocity;           // 最大线速度 [m/s]
    double max_angular_vel;        // 最大角速度 [rad/s]
    double min_velocity;           // 最小线速度 [m/s]

    // 线速度加减速限制（用于更平滑的加速/减速）
    double linear_accel_limit;     // 最大加速度 [m/s^2]
    double linear_decel_limit;     // 最大减速度 [m/s^2]（正数）

    // 控制增益
    double k_linear;               // 线速度增益
    double k_angular;              // 角速度增益

    // 到达判定
    double arrival_tolerance;      // 位置到达容差 [m]
    double arrival_angle_tolerance; // 角度到达容差 [rad]
    int arrival_confirm_count;     // 确认次数

    // 减速控制
    double slow_down_distance;     // 减速区距离 [m]
    double creep_distance;         // 进入蠕动段距离 [m]
    double creep_velocity;         // 蠕动速度 [m/s]

    // 蠕动速度硬下限（近目标阶段强制 |v|>=creep_velocity）
    bool hard_creep_min_enabled;   // 是否启用

    // 大转角处理
    double large_angle_threshold;  // 大转角阈值 [rad]
    double large_angle_vel_ratio;  // 大转角速度比例

    // 姿态伺服
    double pose_servo_distance;    // 姿态伺服切换距离 [m]

    // 阶段1重入控制（从阶段2退回阶段1时的速度限制）
    bool allow_stage2_exit;                 // 是否允许从阶段2退回阶段1
    double stage1_reentry_max_velocity;     // 重入时最大线速度 [m/s]
    double stage1_reentry_max_angular_vel;  // 重入时最大角速度 [rad/s]

    // 阶段2航向对准（原地旋转，对齐终点朝向）
    // 计算方式与 LineFollowController::calculateRotationVelocity 一致（sigmoid + 近零平滑 + min/max 限幅）
    bool rotation_enabled;             // 是否启用 rotation 方式（false 则使用 k_angular 比例控制）
    double rotation_max_w;             // 最大角速度 [rad/s]
    double rotation_min_w;             // 最小角速度 [rad/s]（会被动态缩减）
    double rotation_absolute_min_w;    // 绝对最小角速度 [rad/s]（不会被动态缩减，保证机器人始终能动）
    double rotation_factor;            // sigmoid 因子
    double rotation_angle_threshold;   // 近零平滑阈值 [rad]
    double rotation_smooth_factor;     // 近零平滑系数
    double rotation_stop_tolerance;    // 认为"已对准"的角度阈值 [rad]（建议与 arrival_angle_tolerance 一致）
    bool rotation_use_crossing_stop;   // 是否允许"过零即认为对准"（防止来回抖动）
    double rotation_crossing_window;   // 过零判定窗口 [rad]（仅在误差均小于该值时允许过零判定）

    // 后退模式
    bool enable_backward;          // 是否启用后退模式
    double backward_angle_threshold; // 后退角度阈值 [rad]

    // 平滑控制
    double velocity_smooth_alpha;  // 速度平滑系数

    // 动态速度重置（接收新目标时根据初始距离重置最大线速度）
    double dynamic_vel_reset_max_vel;       // 重置后的最大线速度上限 [m/s]
    double dynamic_vel_reset_min_vel;       // 重置后的最小线速度下限 [m/s]
    double dynamic_vel_threshold_distance;  // 距离阈值 [m]：< 此值直接用 min_vel
    double dynamic_vel_dist_step;           // 距离步长 [m]：超出阈值后每增加此距离
    double dynamic_vel_vel_step;            // 速度步长 [m/s]：对应增加此速度

    // 角速度大时线速度衰减（防止高角速度引起漂移）
    double angular_vel_decel_scale;      // 衰减强度 [0,1]（0=不衰减，1=满角速度时线速度降为0）
    double angular_vel_decel_min_factor; // 衰减后线速度的最小比例（防止被压得太低）

    // 调试
    bool debug_enabled;            // 是否启用调试输出
  };

  // ================================
  // 私有成员变量
  // ================================

  Parameters params_;              // 控制参数

  // 目标状态
  double goal_x_;                  // 目标X坐标
  double goal_y_;                  // 目标Y坐标
  double goal_theta_;              // 目标朝向
  bool goal_set_;                  // 是否设置了目标
  bool goal_theta_set_;            // 是否设置了目标朝向

  // 到达状态
  bool goal_reached_;              // 是否到达目标
  int arrival_count_;              // 连续到达计数

  // 运动模式
  bool use_backward_;              // 是否使用后退模式
  bool backward_decided_;          // 是否已经判断过后退模式
  bool backward_switched_to_forward_; // 是否已从后退切换到前进（防止反复切换）

  // 后退安全检测
  double prev_distance_;           // 上一次距离（用于检测距离增大）
  int distance_increasing_count_;  // 距离增大计数器
  double min_distance_reached_;    // 到达过的最小距离

  // 速度平滑
  double prev_linear_vel_;         // 上一次线速度
  double prev_angular_vel_;        // 上一次角速度

  // 时间管理
  std::chrono::steady_clock::time_point last_time_;
  rclcpp::Time last_log_output_time_; // 上次日志输出时间

  // 动态速度重置标志
  bool first_compute_after_goal_;     // 接收到新目标后是否是第一次 computeVelocityCommands

  // 阶段2控制状态
  double last_stage2_yaw_error_;      // 阶段2航向误差（用于过零判定）
  double stage2_entry_distance_;      // 进入阶段2时的距离（用于放宽位置容差）
  bool stage2_position_relaxed_;      // 是否已进入阶段2并放宽位置容差
  bool stage1_reentry_mode_;          // 是否处于阶段1重入模式（从阶段2退回，需要限制速度）

  // ================================
  // 私有方法
  // ================================

  /**
   * @brief 从YAML加载参数
   */
  void updateParameters();

  /**
   * @brief 计算到目标的距离
   */
  double computeDistance(double curr_x, double curr_y) const;

  /**
   * @brief 计算目标航向
   */
  double computeTargetHeading(double curr_x, double curr_y) const;

  /**
   * @brief 归一化角度到[-π, π]
   */
  double normalizeAngle(double angle) const;

  /**
   * @brief 计算线速度
   */
  double computeLinearVelocity(double distance, double heading_error);

  /**
   * @brief 近目标蠕动速度硬下限（避免 v 被角度衰减/平滑压到 0）
   */
  void enforceHardCreepMin(double& v, double distance, double curr_x, double curr_y, double curr_theta);

  /**
   * @brief 计算角速度
   */
  double computeAngularVelocity(double heading_error);

  /**
   * @brief 阶段2原地对准航向角速度（与 LineFollowController 对齐方式一致）
   */
  double computeRotationVelocity(double angle_diff);

  /**
   * @brief 平滑速度输出
   */
  void smoothVelocity(double& v, double& omega, double dt);

  /**
   * @brief 判断是否应该使用后退模式
   * @param curr_x 当前X坐标
   * @param curr_y 当前Y坐标
   * @param curr_theta 当前朝向
   * @return true表示应该后退
   */
  bool shouldUseBackward(double curr_x, double curr_y, double curr_theta);
};

}  // namespace follow_controller
}  // namespace xline
