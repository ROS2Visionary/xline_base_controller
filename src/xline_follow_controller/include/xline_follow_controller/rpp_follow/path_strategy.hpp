#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace xline
{
namespace follow_controller
{


struct PathStrategyContext
{
  // 当前状态
  geometry_msgs::msg::PoseStamped current_pose;   ///< 当前机器人位姿
  geometry_msgs::msg::Twist current_velocity;      ///< 当前速度
  double current_yaw;                              ///< 当前航向角

  // Pure Pursuit 计算结果
  double angle_to_lookahead;                       ///< 到前瞻点的角度误差
  double lookahead_distance;                       ///< 前瞻距离
  double curvature;                                ///< 当前曲率
  double base_angular_velocity;                    ///< 基础角速度 (v * curvature)
  double desired_linear_velocity;                  ///< 期望线速度

  // 控制参数
  double dt;                                       ///< 控制周期
  double min_v;                                    ///< 最小线速度
  double max_v;                                    ///< 最大线速度

  // 路径信息
  double remaining_distance;                       ///< 剩余距离
  double traversed_distance;                       ///< 已行驶距离
};


// 路径策略结果,策略计算返回的结果数据
struct PathStrategyResult
{
  double angular_velocity;                         ///< 期望角速度
  double linear_velocity;                          ///< 期望线速度
  bool goal_reached;                               ///< 是否到达目标
  bool filter_reset;                               ///< 是否需要重置滤波器
  std::string status_message;                      ///< 状态信息（用于日志）
};


// 通过继承此接口实现各自的特有逻辑。
class PathStrategy
{
public:
  using SharedPtr = std::shared_ptr<PathStrategy>;
  using UniquePtr = std::unique_ptr<PathStrategy>;

  PathStrategy() = default;
  virtual ~PathStrategy() = default;

  // 禁止拷贝
  PathStrategy(const PathStrategy&) = delete;
  PathStrategy& operator=(const PathStrategy&) = delete;

  // 核心接口
  virtual std::string getTypeName() const = 0;
  virtual bool setPlan(const nav_msgs::msg::Path& path) = 0;
  virtual bool isGoalReached(const PathStrategyContext& ctx) = 0;
  virtual void computeAngularVelocity(const PathStrategyContext& ctx,
                                       PathStrategyResult& result) = 0;
  virtual void reset() = 0;
  virtual void updateParameters(const std::string& config_path) = 0;

  // 可选接口（带默认实现）
  virtual double getTargetYaw() const
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  virtual bool needsYawPrealignment() const
  {
    return false;
  }

  virtual void setYawPrealignmentDone()
  {
  }

  virtual std::string getDebugInfo() const
  {
    return "";
  }

  void setLogger(const rclcpp::Logger& logger)
  {
    logger_ = logger;
    has_logger_ = true;
  }

protected:
  const rclcpp::Logger& getLogger() const
  {
    if (has_logger_)
    {
      return logger_;
    }
    static rclcpp::Logger default_logger = rclcpp::get_logger("path_strategy");
    return default_logger;
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("path_strategy");
  bool has_logger_ = false;
};

// 路径策略类型枚举
enum class PathStrategyType
{
  CURVE,    ///< 曲线路径
  CIRCLE    ///< 圆形路径
};


// 路径策略工厂函数声明
PathStrategy::UniquePtr createPathStrategy(PathStrategyType type);

}  // namespace follow_controller
}  // namespace xline
