#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <xline_msgs/action/execute_plan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <json/json.h>
#include <xline_follow_controller/line_follow_controller.hpp>
#include <xline_follow_controller/rpp_follow_controller.hpp>

/**
 * MotionControlCenter 运动控制中心
 *
 * 该类作为一个独立的 ROS 2 节点，提供 ExecutePlan 动作（Action）服务，
 * 用于接收上位系统下发的执行计划（JSON 字符串形式）并进行执行。
 *
 * - 动作名：execute_plan
 * - 目标（Goal）字段：plan_json, plan_uid
 * - 反馈（Feedback）：current_order, status（空闲/执行中/暂停）
 * - 结果（Result）：success, error_message
 */
namespace xline
{
namespace base_controller
{

class MotionControlCenter : public rclcpp::Node
{
public:
  using ExecutePlan = xline_msgs::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ServerGoalHandle<ExecutePlan>;

  /**
   * 构造函数
   * @param options 节点选项
   */
  explicit MotionControlCenter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MotionControlCenter() override;

private:
  // ExecutePlan 动作服务器实例
  rclcpp_action::Server<ExecutePlan>::SharedPtr action_server_;
  xline::follow_controller::BaseFollowController::SharedPtr base_follow_controller_;
  xline::follow_controller::LineFollowController::SharedPtr line_follow_controller_;
  xline::follow_controller::RPPController::SharedPtr rpp_follow_controller_;

  // 位姿订阅器(从状态估计器获取融合后的位姿)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  // 最新位姿缓存与同步
  geometry_msgs::msg::PoseStamped latest_pose_;
  std::atomic<bool> has_latest_pose_{false};
  std::mutex pose_mutex_;

  // 执行状态标志及互斥锁
  std::atomic<bool> is_executing_{false};
  std::mutex execution_mutex_;

  /**
   * 目标处理回调：决定是否接受/拒绝目标
   * @param uuid  目标UUID
   * @param goal  目标内容（包含 plan_json / plan_uid）
   */
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid,
                                         std::shared_ptr<const ExecutePlan::Goal> goal);

  /**
   * 取消处理回调：收到客户端取消时的响应
   */
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  /**
   * 接受目标回调：在单独线程中执行 execute()
   */
  void handleAccepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  /**
   * 执行主循环：发布反馈、响应取消、完成结果
   */
  void execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  /**
   * 位姿数据回调函数(接收融合后的位姿)
   */
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // 数据结构定义
  struct LineData {
    double start_x;
    double start_y;
    double end_x;
    double end_y;
  };

  struct CircleData {
    double center_x;
    double center_y;
    double radius;
  };

  struct ArcData {
    double center_x;
    double center_y;
    double radius;
    double start_angle;  // 弧度
    double end_angle;    // 弧度
  };

  /**
   * 提取line数据
   */
  LineData extractLineData(const Json::Value & line_obj);

  /**
   * 提取circle数据
   */
  CircleData extractCircleData(const Json::Value & circle_obj);

  /**
   * 提取arc数据
   */
  ArcData extractArcData(const Json::Value & arc_obj);
};

}  // namespace base_controller
}  // namespace xline
