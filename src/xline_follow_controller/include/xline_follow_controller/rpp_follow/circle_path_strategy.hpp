/**
 * @file circle_path_strategy.hpp
 * @brief 圆形路径跟随策略声明
 *
 * 实现圆形路径的 Pure Pursuit 跟随控制，使用累计角度判定目标到达，
 * 并对角速度进行基于基准值的偏差限制。
 */

#pragma once

#include "xline_follow_controller/rpp_follow/path_strategy.hpp"
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
   *
   * 以圆心+半径+起始位姿生成圆弧路径
   *
   * @param center_x 圆心 x 坐标
   * @param center_y 圆心 y 坐标
   * @param radius 圆半径
   * @param start_pose 起始位姿（切入点）
   * @return 生成的路径
   */
  nav_msgs::msg::Path generateCirclePath(double center_x, double center_y,
                                          double radius,
                                          const geometry_msgs::msg::PoseStamped& start_pose);

  /**
   * @brief 设置圆弧角度范围
   * @param start_angle 起始角度（弧度）
   * @param end_angle 结束角度（弧度）
   */
  void setAngleRange(double start_angle, double end_angle);

  /**
   * @brief 设置圆心坐标
   * @param x 圆心 x
   * @param y 圆心 y
   */
  void setCircleCenter(double x, double y);

  /**
   * @brief 设置圆半径
   * @param radius 半径（米）
   */
  void setCircleRadius(double radius);

  /**
   * @brief 获取基准角速度
   * @return 基准角速度 (v / r)
   */
  double getBaselineAngularVelocity() const { return baseline_angular_velocity_; }

  /**
   * @brief 获取累计角度
   * @return 当前累计转角（弧度）
   */
  double getAccumulatedAngle() const { return accumulated_angle_; }

  /**
   * @brief 获取目标总角度
   * @return 目标总转角（弧度）
   */
  double getTotalAngle() const { return circle_total_angle_; }

  /**
   * @brief 获取圆心坐标
   * @param x 输出圆心 x
   * @param y 输出圆心 y
   */
  void getCircleCenter(double& x, double& y) const
  {
    x = circle_center_x_;
    y = circle_center_y_;
  }

  /**
   * @brief 获取圆半径
   * @return 圆半径（米）
   */
  double getCircleRadius() const { return circle_radius_; }

  /**
   * @brief 根据半径调整速度参数
   * @param radius 圆半径
   * @param min_v 输出调整后的最小速度
   * @param max_v 输出调整后的最大速度
   * @param lookahead_dist 输出调整后的前瞻距离
   */
  void adjustSpeedForRadius(double radius, double& min_v, double& max_v,
                             double& lookahead_dist) const;

  /**
   * @brief 设置基准线速度（用于计算基准角速度）
   * @param min_v 基准线速度
   */
  void setBaselineLinearVelocity(double min_v);

  /**
   * @brief 获取用于打印控制的标志
   */
  bool shouldStartPrint() const { return start_print_; }
  bool shouldStopPrint() const { return stop_print_; }

private:
  // ================================
  // 圆形路径参数
  // ================================

  double circle_center_x_;           ///< 圆心 x 坐标
  double circle_center_y_;           ///< 圆心 y 坐标
  double circle_radius_;             ///< 圆半径
  double circle_entry_x_;            ///< 切入点 x 坐标
  double circle_entry_y_;            ///< 切入点 y 坐标
  double circle_start_angle_;        ///< 圆弧起始角度
  double circle_end_angle_;          ///< 圆弧结束角度
  double circle_total_angle_;        ///< 圆弧总角度
  double baseline_angular_velocity_; ///< 基准角速度 (v / r)

  // ================================
  // 角度累计相关
  // ================================

  bool last_yaw_initialized_;        ///< 上次航向角是否已初始化
  double last_yaw_;                  ///< 上次航向角
  double accumulated_angle_;         ///< 累计转角
  int angle_debug_counter_;          ///< 调试计数器

  // ================================
  // 航向预对准
  // ================================

  bool need_yaw_prealign_;           ///< 是否需要预对准
  bool yaw_prealign_done_;           ///< 预对准是否完成
  double target_yaw_;                ///< 目标航向角

  // ================================
  // 角速度偏差参数
  // ================================

  double start_deviation_factor_;    ///< 起始偏差因子
  double end_deviation_factor_;      ///< 结束偏差因子
  double deviation_rate_;            ///< 偏差率

  // ================================
  // 其他参数
  // ================================

  double goal_dist_tol_;             ///< 目标距离容忍度
  double rotate_tol_;                ///< 角度容忍度
  double radius_offset_;             ///< 半径偏移

  // ================================
  // 状态
  // ================================

  nav_msgs::msg::Path global_plan_;  ///< 全局路径
  bool goal_reached_;                ///< 是否到达目标

  // 打印控制标志
  bool start_print_;
  bool stop_print_;

  // ================================
  // 私有方法
  // ================================

  /**
   * @brief 重置圆形路径状态变量
   */
  void resetCirclePathState();

  /**
   * @brief 更新累计角度
   * @param current_yaw 当前航向角
   * @return 是否完成
   */
  bool updateAccumulatedAngle(double current_yaw);

  /**
   * @brief 计算带偏差约束的角速度
   * @param base_omega 基础角速度
   * @return 约束后的角速度
   */
  double constrainAngularVelocity(double base_omega);
};

}  // namespace follow_controller
}  // namespace xline
