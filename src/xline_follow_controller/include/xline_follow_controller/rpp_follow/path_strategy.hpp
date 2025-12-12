/**
 * @file path_strategy.hpp
 * @brief 路径跟随策略接口定义
 *
 * 定义路径策略的抽象接口，支持圆形路径和曲线路径两种实现。
 * 使用策略模式分离不同路径类型的特有逻辑。
 */

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

/**
 * @brief 路径策略上下文信息
 *
 * 用于在控制器和策略之间传递计算所需的上下文数据
 */
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

/**
 * @brief 路径策略结果
 *
 * 策略计算返回的结果数据
 */
struct PathStrategyResult
{
  double angular_velocity;                         ///< 期望角速度
  double linear_velocity;                          ///< 期望线速度
  bool goal_reached;                               ///< 是否到达目标
  bool filter_reset;                               ///< 是否需要重置滤波器
  std::string status_message;                      ///< 状态信息（用于日志）
};

/**
 * @brief 路径策略抽象基类
 *
 * 定义路径跟随策略的通用接口，不同类型的路径（圆形、曲线等）
 * 通过继承此接口实现各自的特有逻辑。
 */
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

  // ================================
  // 核心接口
  // ================================

  /**
   * @brief 获取策略类型名称
   * @return 策略类型标识字符串
   */
  virtual std::string getTypeName() const = 0;

  /**
   * @brief 设置路径计划
   * @param path 全局路径
   * @return 设置成功返回 true
   */
  virtual bool setPlan(const nav_msgs::msg::Path& path) = 0;

  /**
   * @brief 检查是否到达目标
   * @param ctx 当前上下文信息
   * @return 到达目标返回 true
   */
  virtual bool isGoalReached(const PathStrategyContext& ctx) = 0;

  /**
   * @brief 计算角速度
   *
   * 根据策略特有的逻辑调整角速度输出
   *
   * @param ctx 当前上下文信息
   * @param result 输出结果
   */
  virtual void computeAngularVelocity(const PathStrategyContext& ctx,
                                       PathStrategyResult& result) = 0;

  /**
   * @brief 重置策略状态
   *
   * 清空内部状态，准备处理新的路径
   */
  virtual void reset() = 0;

  /**
   * @brief 更新策略参数
   * @param params YAML 参数解析器（具体类型由实现决定）
   */
  virtual void updateParameters(const std::string& config_path) = 0;

  // ================================
  // 可选接口（带默认实现）
  // ================================

  /**
   * @brief 获取目标航向角（用于预对准）
   * @return 目标航向角（弧度），如果不需要预对准返回 NaN
   */
  virtual double getTargetYaw() const
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * @brief 是否需要航向预对准
   * @return 需要预对准返回 true
   */
  virtual bool needsYawPrealignment() const
  {
    return false;
  }

  /**
   * @brief 设置预对准完成状态
   */
  virtual void setYawPrealignmentDone()
  {
    // 默认空实现
  }

  /**
   * @brief 获取用于调试的状态信息
   * @return 状态描述字符串
   */
  virtual std::string getDebugInfo() const
  {
    return "";
  }

  /**
   * @brief 设置 ROS 日志器
   * @param logger ROS 日志器实例
   */
  void setLogger(const rclcpp::Logger& logger)
  {
    logger_ = logger;
    has_logger_ = true;
  }

protected:
  /**
   * @brief 获取日志器
   * @return ROS 日志器引用
   */
  const rclcpp::Logger& getLogger() const
  {
    if (has_logger_)
    {
      return logger_;
    }
    static rclcpp::Logger default_logger = rclcpp::get_logger("path_strategy");
    return default_logger;
  }

  /**
   * @brief 角度归一化到 [-π, π]
   * @param angle 输入角度（弧度）
   * @return 归一化后的角度
   */
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

/**
 * @brief 路径策略类型枚举
 */
enum class PathStrategyType
{
  CURVE,    ///< 曲线路径
  CIRCLE    ///< 圆形路径
};

/**
 * @brief 路径策略工厂函数声明
 * @param type 策略类型
 * @return 策略实例指针
 */
PathStrategy::UniquePtr createPathStrategy(PathStrategyType type);

}  // namespace follow_controller
}  // namespace xline
