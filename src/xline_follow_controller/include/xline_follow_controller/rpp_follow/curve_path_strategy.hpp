#pragma once

#include "xline_follow_controller/rpp_follow/path_strategy.hpp"
#include "xline_follow_controller/rpp_follow/rpp_params.hpp"
#include <nav_msgs/msg/path.hpp>

namespace xline
{
namespace follow_controller
{


class CurvePathStrategy : public PathStrategy
{
public:
  CurvePathStrategy();
  ~CurvePathStrategy() override = default;

  // PathStrategy 接口实现
  std::string getTypeName() const override { return "curve"; }

  bool setPlan(const nav_msgs::msg::Path& path) override;

  bool isGoalReached(const PathStrategyContext& ctx) override;

  void computeAngularVelocity(const PathStrategyContext& ctx,
                               PathStrategyResult& result) override;

  void reset() override;

  void updateParameters(const std::string& config_path) override;

  std::string getDebugInfo() const override;

  // 曲线路径特有接口
  void setBackFollow(bool enable);
  bool isBackFollow() const { return back_follow_; }
  const geometry_msgs::msg::PoseStamped& getGoalPose() const { return goal_pose_; }
  double getPathLength() const { return path_length_; }

private:


  RPPPathStrategyParams params_;    ///< 策略参数

  // 状态
  nav_msgs::msg::Path global_plan_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  double goal_x_, goal_y_, goal_theta_;
  double path_length_;
  bool goal_reached_;
  bool back_follow_;

  // 私有方法
  void calculatePathInfo();
  double applyApproachConstraint(double raw_velocity, double distance_to_goal);
};

}  // namespace follow_controller
}  // namespace xline
