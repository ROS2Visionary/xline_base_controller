/**
 * @file circle_path_strategy.hpp
 * @brief 圆形路径跟随策略声明（结构体参数版）
 *
 * 实现圆形路径的 Pure Pursuit 跟随控制，使用累计角度判定目标到达，
 * 并对角速度进行基于基准值的偏差限制。
 */

#pragma once

#include "xline_follow_controller/rpp_follow/path_strategy.hpp"
#include "xline_follow_controller/rpp_follow/rpp_params.hpp"
#include <nav_msgs/msg/path.hpp>

namespace xline
{
namespace follow_controller
{

/**
 * @brief 圆形路径跟随策略
 *
 * 适用于圆弧路径的特殊控制策略：
 * - 自动生成切入直线 + 圆弧路径
 * - 基于累计航向角变化的目标到达判定
 * - 基于基准角速度的偏差约束
 * - 根据半径自动调整速度参数
 * - 支持航向预对准
 */
class CirclePathStrategy : public PathStrategy
{
public:
  CirclePathStrategy();
  ~CirclePathStrategy() override = default;

  // ================================
  // PathStrategy 接口实现
  // ================================

  std::string getTypeName() const override { return "circle"; }

  bool setPlan(const nav_msgs::msg::Path& path) override;

  bool isGoalReached(const PathStrategyContext& ctx) override;

  void computeAngularVelocity(const PathStrategyContext& ctx,
                               PathStrategyResult& result) override;

  void reset() override;

  void updateParameters(const std::string& config_path) override;

  double getTargetYaw() const override { return target_yaw_; }

  bool needsYawPrealignment() const override { return need_yaw_prealign_ && !yaw_prealign_done_; }

  void setYawPrealignmentDone() override;

  std::string getDebugInfo() const override;

  // ================================
  // 圆形路径特有接口
  // ================================

  /**
   * @brief 设置圆形路径参数并生成路径
   */
  nav_msgs::msg::Path generateCirclePath(double center_x, double center_y,
                                          double radius,
                                          const geometry_msgs::msg::PoseStamped& start_pose);

  void setAngleRange(double start_angle, double end_angle);
  void setCircleCenter(double x, double y);
  void setCircleRadius(double radius);

  double getBaselineAngularVelocity() const { return baseline_angular_velocity_; }
  double getAccumulatedAngle() const { return accumulated_angle_; }
  double getTotalAngle() const { return circle_total_angle_; }

  void getCircleCenter(double& x, double& y) const
  {
    x = circle_center_x_;
    y = circle_center_y_;
  }

  double getCircleRadius() const { return circle_radius_; }

  void adjustSpeedForRadius(double radius, double& min_v, double& max_v,
                             double& lookahead_dist) const;

  void setBaselineLinearVelocity(double min_v);

  bool shouldStartPrint() const { return start_print_; }
  bool shouldStopPrint() const { return stop_print_; }

private:
  // ================================
  // 参数结构体
  // ================================
  
  RPPPathStrategyParams params_;   ///< 策略参数

  // ================================
  // 圆形路径参数
  // ================================

  double circle_center_x_;
  double circle_center_y_;
  double circle_radius_;
  double circle_entry_x_;
  double circle_entry_y_;
  double circle_start_angle_;
  double circle_end_angle_;
  double circle_total_angle_;
  double baseline_angular_velocity_;

  // ================================
  // 角度累计相关
  // ================================

  bool last_yaw_initialized_;
  double last_yaw_;
  double accumulated_angle_;
  int angle_debug_counter_;

  // ================================
  // 航向预对准
  // ================================

  bool need_yaw_prealign_;
  bool yaw_prealign_done_;
  double target_yaw_;

  // ================================
  // 状态
  // ================================

  nav_msgs::msg::Path global_plan_;
  bool goal_reached_;
  bool start_print_;
  bool stop_print_;

  // ================================
  // 私有方法
  // ================================

  void resetCirclePathState();
  bool updateAccumulatedAngle(double current_yaw);
  double constrainAngularVelocity(double base_omega);
};

}  // namespace follow_controller
}  // namespace xline
