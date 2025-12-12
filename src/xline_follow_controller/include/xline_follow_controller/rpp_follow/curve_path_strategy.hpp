#pragma once

#include "xline_follow_controller/rpp_follow/path_strategy.hpp"
#include "xline_follow_controller/rpp_follow/rpp_params.hpp"
#include <nav_msgs/msg/path.hpp>

namespace xline
{
namespace follow_controller
{

/**
 * @brief 曲线路径跟随策略
 *
 * 用于处理一般的“曲线/折线”路径，主要职责有：
 * - 接收一条全局路径，做基础的长度统计和终点信息记录；
 * - 根据当前位置判断是否到达路径终点（距离阈值判定）；
 * - 在接近终点时对线速度做“靠近约束”，保证减速平滑；
 * - 向外暴露路径长度、目标位姿和是否后退等状态。
 *
 * 实现 PathStrategy 接口，由 RPPController 在不同场景下切换使用。
 */
class CurvePathStrategy : public PathStrategy
{
public:
  CurvePathStrategy();
  ~CurvePathStrategy() override = default;

  // ================================
  // PathStrategy 接口实现
  // ================================

  /// 策略类型名称，用于日志和调试
  std::string getTypeName() const override { return "curve"; }

  /// 设置当前要跟随的路径
  bool setPlan(const nav_msgs::msg::Path& path) override;

  /// 根据当前位置与目标点的距离判断是否到达终点
  bool isGoalReached(const PathStrategyContext& ctx) override;

  /// 计算本步的角速度和线速度（内部只调整线速度）
  void computeAngularVelocity(const PathStrategyContext& ctx,
                               PathStrategyResult& result) override;

  /// 重置内部状态（清空路径和目标信息）
  void reset() override;

  /// 从 YAML 配置文件中加载路径相关参数（距离阈值等）
  void updateParameters(const std::string& config_path) override;

  /// 返回一串包含长度、终点等信息的调试字符串
  std::string getDebugInfo() const override;

  // ================================
  // 曲线路径特有接口
  // ================================

  /// 设置是否启用后退模式（仅记录标志，具体控制由上层处理）
  void setBackFollow(bool enable);

private:
  /// 与 YAML 中结构对应的策略参数（目标、靠近约束等）
  RPPPathStrategyParams params_;    ///< 策略参数

  // 路径和状态
  nav_msgs::msg::Path global_plan_;           ///< 当前跟随的全局路径
  geometry_msgs::msg::PoseStamped goal_pose_; ///< 路径终点位姿
  double goal_x_, goal_y_, goal_theta_;       ///< 终点位置和朝向（缓存）
  double path_length_;                        ///< 路径总长度
  bool goal_reached_;                         ///< 是否已到达终点
  bool back_follow_;                          ///< 是否启用后退跟随

  // ================================
  // 内部辅助方法
  // ================================

  /// 从路径计算长度和终点信息
  void calculatePathInfo();

  /// 接近终点时对线速度做约束，越接近速度越小
  double applyApproachConstraint(double raw_velocity, double distance_to_goal);
};

}  // namespace follow_controller
}  // namespace xline
