/**
 * @file curve_path_strategy.hpp
 * @brief 曲线路径跟随策略声明
 *
 * 实现标准 Pure Pursuit 曲线路径跟随，使用距离判定目标到达。
 */

#pragma once

#include "xline_follow_controller/rpp_follow/path_strategy.hpp"
#include <nav_msgs/msg/path.hpp>

namespace xline
{
namespace follow_controller
{

/**
 * @brief 曲线路径跟随策略
 *
 * 适用于一般曲线路径的 Pure Pursuit 跟随控制：
 * - 基于距离的目标到达判定
 * - 标准 Pure Pursuit 角速度计算
 * - 支持后退跟随模式
 */
class CurvePathStrategy : public PathStrategy
{
public:
  CurvePathStrategy();
  ~CurvePathStrategy() override = default;

  // ================================
  // PathStrategy 接口实现
  // ================================

  std::string getTypeName() const override { return "curve"; }

  bool setPlan(const nav_msgs::msg::Path& path) override;

  bool isGoalReached(const PathStrategyContext& ctx) override;

  void computeAngularVelocity(const PathStrategyContext& ctx,
                               PathStrategyResult& result) override;

  void reset() override;

  void updateParameters(const std::string& config_path) override;

  std::string getDebugInfo() const override;

  // ================================
  // 曲线路径特有接口
  // ================================

  /**
   * @brief 设置后退跟随模式
   * @param enable true 启用后退模式
   */
  void setBackFollow(bool enable);

  /**
   * @brief 获取当前是否为后退模式
   * @return 后退模式返回 true
   */
  bool isBackFollow() const { return back_follow_; }

  /**
   * @brief 获取目标位姿
   * @return 路径终点位姿
   */
  const geometry_msgs::msg::PoseStamped& getGoalPose() const { return goal_pose_; }

  /**
   * @brief 获取路径长度
   * @return 路径总长度（米）
   */
  double getPathLength() const { return path_length_; }

private:
  // ================================
  // 参数
  // ================================

  double goal_dist_tol_;      ///< 目标距离容忍度（米）
  double rotate_tol_;         ///< 目标角度容忍度（弧度）
  double approach_dist_;      ///< 接近目标距离阈值
  double approach_min_v_;     ///< 接近目标最小速度

  // ================================
  // 状态
  // ================================

  nav_msgs::msg::Path global_plan_;              ///< 全局路径
  geometry_msgs::msg::PoseStamped goal_pose_;    ///< 目标位姿
  double goal_x_, goal_y_, goal_theta_;          ///< 目标位置和角度
  double path_length_;                           ///< 路径总长度
  bool goal_reached_;                            ///< 是否到达目标
  bool back_follow_;                             ///< 后退模式标志

  // ================================
  // 私有方法
  // ================================

  /**
   * @brief 计算路径信息（长度、目标点等）
   */
  void calculatePathInfo();

  /**
   * @brief 应用接近目标时的速度约束
   * @param raw_velocity 原始速度
   * @param distance_to_goal 到目标的距离
   * @return 约束后的速度
   */
  double applyApproachConstraint(double raw_velocity, double distance_to_goal);
};

}  // namespace follow_controller
}  // namespace xline
