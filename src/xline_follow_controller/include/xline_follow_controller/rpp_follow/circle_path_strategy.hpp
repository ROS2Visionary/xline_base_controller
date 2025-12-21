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
 * 这个策略只负责“围绕某个圆心转一圈（或多圈）”的控制逻辑：
 * - 根据圆心、半径和起始位姿生成圆弧路径；
 * - 累计已经旋转的角度，用于判断是否完成一圈；
 * - 根据半径和偏差参数，对角速度做一定限幅和修正；
 * - 暴露一些用于打印控制的状态（开始、结束等）。
 *
 * 和 CurvePathStrategy 一样，它实现 PathStrategy 接口，供 RPPController 选择使用。
 */
class CirclePathStrategy : public PathStrategy
{
public:
  CirclePathStrategy();
  ~CirclePathStrategy() override = default;

  // ================================
  // PathStrategy 接口实现
  // ================================

  /// 策略类型名称，用于日志和调试
  std::string getTypeName() const override { return "circle"; }

  /// 设置当前要跟随的路径（通常是 generateCirclePath 生成的结果）
  bool setPlan(const nav_msgs::msg::Path& path) override;

  /// 根据累计旋转角度判断是否“走完一圈”
  bool isGoalReached(const PathStrategyContext& ctx) override;

  /// 计算当前时刻的角速度、线速度及滤波器重置标志
  void computeAngularVelocity(const PathStrategyContext& ctx,
                               PathStrategyResult& result) override;

  // ========================================
  // 动态参数接口实现（圆形路径恒定参数）
  // ========================================

  bool hasDynamicVelocity() const override
  {
    return true;
  }

  double getTargetVelocity() const override
  {
    return circle_target_velocity_;
  }

  double getTargetLookahead() const override
  {
    return circle_target_lookahead_;
  }

  double getTargetAngularVelocity() const override
  {
    return circle_target_angular_velocity_;
  }

  void initializeDynamicParams() override;

  /// 将内部状态重置到初始状态，但不改参数
  void reset() override;

  /// 从 YAML 配置文件中加载策略相关参数
  void updateParameters(const std::string& config_path) override;

  /// 返回进入圆弧时希望对齐的航向角
  double getTargetYaw() const override { return target_yaw_; }

  /// 是否需要在进入圆轨迹前做一次航向预对准
  bool needsYawPrealignment() const override { return need_yaw_prealign_ && !yaw_prealign_done_; }

  /// 通知策略：上层已经完成了航向预对准
  void setYawPrealignmentDone() override;

  /// 返回一串用于调试的状态字符串
  std::string getDebugInfo() const override;


  // ================================
  // 圆形路径相关的公开接口
  // ================================

  /**
   * @brief 设置圆形参数并生成一条圆弧路径
   *
   * @param center_x 圆心 X 坐标
   * @param center_y 圆心 Y 坐标
   * @param radius   圆半径
   * @param start_pose 起始位姿（通常是机器人当前位姿）
   * @return 生成的圆弧路径（包含切入点和均匀采样的圆弧点）
   */
  nav_msgs::msg::Path generateCirclePath(double center_x, double center_y,
                                          double radius,
                                          const geometry_msgs::msg::PoseStamped& start_pose);

  /// 设置起止角度范围（单位：rad），用于计算总目标角度
  void setAngleRange(double start_angle, double end_angle);

  /// 设置圆心坐标
  void setCircleCenter(double x, double y);

  /// 设置圆半径
  void setCircleRadius(double radius);

  /// 获取基准角速度（基于最小线速度和半径计算）
  double getBaselineAngularVelocity() const { return baseline_angular_velocity_; }

  /// 获取当前圆半径（供外部查询）
  double getCircleRadius() const { return circle_radius_; }

  /**
   * @brief 设置基准线速度，并同步出一个对应的基准角速度
   *
   * 后续的角速度限制会围绕这个基准值做一定比例内的调整。
   */
  void setBaselineLinearVelocity(double min_v);

  /// 是否应该开始打印（累计角度达到一定阈值后置为 true）
  bool shouldStartPrint() const { return start_print_; }

  /// 是否应该停止打印（圆轨迹结束前一小段会触发）
  bool shouldStopPrint() const { return stop_print_; }

private:
  /// 与 YAML 中结构对应的策略参数（目标、滤波、偏差控制等）
  RPPPathStrategyParams params_;   ///< 策略参数

  // 圆形路径参数（几何信息）
  double circle_center_x_;
  double circle_center_y_;
  double circle_radius_;
  double circle_total_angle_;
  double baseline_angular_velocity_;

  // 动态计算的恒定参数（在 setPlanForCircle 时初始化）
  double circle_target_velocity_ = 0.0;         ///< 目标线速度
  double circle_target_lookahead_ = 0.0;        ///< 目标前瞻距离
  double circle_target_angular_velocity_ = 0.0; ///< 目标角速度 (v/r)

  // 角度累计相关
  bool last_yaw_initialized_;
  double last_yaw_;
  double accumulated_angle_;

  // 航向预对准状态
  bool need_yaw_prealign_;
  bool yaw_prealign_done_;
  double target_yaw_;

  // 当前路径与执行状态
  nav_msgs::msg::Path global_plan_;
  bool goal_reached_;
  bool start_print_;
  bool stop_print_;

  // 打印区间的角度窗口（基于累计角度）
  bool print_window_initialized_ = false;
  double start_print_angle_ = 0.0;        ///< 触发“开始打印信号”时的累计角度（喷码机有延时）
  double stop_print_start_angle_ = 0.0;   ///< 停止打印开始点（关闭打印起点）
  double stop_print_end_angle_ = 0.0;     ///< 停止打印结束点（到达该角度即认为圆弧完成）

  // ================================
  // 内部辅助方法
  // ================================

  /// 重置角度累计相关的内部状态
  void resetCirclePathState();

  /// 增量更新累计角度，并判断是否达到目标角度
  bool updateAccumulatedAngle(double current_yaw);

  /// 在基准角速度附近做限幅，避免角速度过大或震荡
  double constrainAngularVelocity(double base_omega);

  /// 计算目标速度（动态或固定）
  double computeTargetVelocity(double radius) const;

  /// 计算目标前瞻距离（使用目标速度而非实时速度）
  double computeTargetLookahead(double velocity, double radius) const;
};

}  // namespace follow_controller
}  // namespace xline
