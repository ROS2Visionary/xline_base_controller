#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <xline_msgs/action/execute_plan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <json/json.h>
#include <xline_follow_controller/line_follow_controller.hpp>
#include <xline_follow_controller/rpp_follow_controller.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
      explicit MotionControlCenter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
      ~MotionControlCenter() override;

      /**
       * 执行定位系统校准
       * 协调机器人移动和调用定位节点的校准服务
       * @param linear_velocity 校准时的前进速度(m/s)，默认0.2
       * @param duration 移动持续时间(秒)，默认10
       * @return 校准是否成功
       */
      bool executeLocalizationCalibration(double linear_velocity = 0.2, double duration = 10.0);

    private:
      // ExecutePlan 动作服务器实例
      rclcpp_action::Server<ExecutePlan>::SharedPtr action_server_;

      std::shared_ptr<xline::follow_controller::BaseFollowController> base_follow_controller_;
      std::shared_ptr<xline::follow_controller::LineFollowController> line_follow_controller_;
      std::shared_ptr<xline::follow_controller::RPPController> rpp_follow_controller_;

      // 位姿订阅器(从状态估计器获取融合后的位姿)
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
      // 最新位姿缓存与同步
      geometry_msgs::msg::PoseStamped latest_pose_;
      std::atomic<bool> has_latest_pose_{false};
      std::mutex pose_mutex_;

      // cmd_vel 发布器(用于校准时控制机器人移动)
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

      // 定位校准服务客户端
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibration_client_;

      // 暂停/恢复服务
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;

      // 执行状态标志及互斥锁
      std::atomic<bool> is_executing_{false};
      std::atomic<bool> is_paused_{false};
      std::condition_variable pause_cv_;
      std::mutex pause_mutex_;

      /**
       * 目标处理回调：决定是否接受/拒绝目标
       * @param uuid  目标UUID
       * @param goal  目标内容（包含 plan_json / plan_uid）
       */
      rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
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

      /**
       * 暂停执行服务回调
       */
      void handlePauseService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);

      /**
       * 恢复执行服务回调
       */
      void handleResumeService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);

      /**
       * 检查并处理暂停状态
       * 如果已暂停，则阻塞等待恢复或取消
       * @param goal_handle 用于检查取消状态
       */
      void checkPauseState(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

      // 数据结构定义
      struct LineData
      {
        double start_x;
        double start_y;
        double end_x;
        double end_y;
      };

      struct CircleData
      {
        double center_x;
        double center_y;
        double radius;
      };

      struct ArcData
      {
        double center_x;
        double center_y;
        double radius;
        double start_angle; // 弧度
        double end_angle;   // 弧度
      };

      /**
       * 提取line数据
       */
      LineData extractLineData(const Json::Value &line_obj);

      /**
       * 提取circle数据
       */
      CircleData extractCircleData(const Json::Value &circle_obj);

      /**
       * 提取arc数据
       */
      ArcData extractArcData(const Json::Value &arc_obj);

      /**
       * 线程安全地获取最新机器人位姿
       * @param[out] pose 拷贝输出的最新位姿
       * @return 若已有缓存位姿并成功拷贝则返回true，否则返回false
       */
      bool getLatestPose(geometry_msgs::msg::PoseStamped & pose);

      // 计算速度控制输出
      // 新增参数：传入 ExecutePlan::Result 的共享指针，
      // 以便在计算过程中可根据需要更新结果信息
      // goal_handle: 用于检查取消状态
      bool compute_velocity(const std::shared_ptr<GoalHandleExecutePlan> goal_handle,
                           ExecutePlan::Result::SharedPtr result);
    };

  } // namespace base_controller
} // namespace xline
