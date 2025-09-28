#include "xline_base_controller/motion_control_center.hpp"

// 为 std::bind 预留占位符
using std::placeholders::_1;
using std::placeholders::_2;

namespace xline
{
namespace base_controller
{

/**
 * 构造函数
 * - 设置节点名称为 "motion_control_center"
 * - 创建 ExecutePlan 动作服务器，动作名为 "execute_plan"
 */
MotionControlCenter::MotionControlCenter(const rclcpp::NodeOptions & options)
  : rclcpp::Node("motion_control_center", options)
{
  // 创建 ExecutePlan 动作服务器：分别绑定 目标处理/取消处理/接受处理 回调
  action_server_ = rclcpp_action::create_server<ExecutePlan>(
      this,                                  // 节点指针
      "execute_plan",                       // 动作名
      std::bind(&MotionControlCenter::handleGoal, this, _1, _2),
      std::bind(&MotionControlCenter::handleCancel, this, _1),
      std::bind(&MotionControlCenter::handleAccepted, this, _1));

  RCLCPP_INFO(get_logger(), "MotionControlCenter 动作服务器已就绪: 'execute_plan'");
}

MotionControlCenter::~MotionControlCenter() = default;

/**
 * 目标处理回调：
 * - 基础合法性检查（plan_uid / plan_json 不能为空）
 * - 通过则返回 ACCEPT_AND_EXECUTE，进入执行流程
 */
rclcpp_action::GoalResponse
MotionControlCenter::handleGoal(const rclcpp_action::GoalUUID & uuid,
                                std::shared_ptr<const ExecutePlan::Goal> goal)
{
  (void)uuid;
  // 基础校验：goal 不能为空
  if (!goal) {
    RCLCPP_WARN(get_logger(), "拒绝目标：收到空的 goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // 内容校验：plan_uid 与 plan_json 不能为空
  if (goal->plan_uid.empty() || goal->plan_json.empty()) {
    RCLCPP_WARN(get_logger(), "拒绝目标：plan_uid 或 plan_json 为空");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "接受目标：plan_uid=%s, json长度=%zu",
              goal->plan_uid.c_str(), goal->plan_json.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * 取消处理回调：
 * - 当前示例中无条件接受取消请求
 */
rclcpp_action::CancelResponse
MotionControlCenter::handleCancel(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "收到取消请求，接受取消");
  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * 接受目标回调：
 * - 为避免阻塞 executor，启动独立线程执行 execute()
 */
void MotionControlCenter::handleAccepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  std::thread{std::bind(&MotionControlCenter::execute, this, goal_handle)}.detach();
}

/**
 * 执行主循环：
 * - 发布初始反馈（IDLE）
 * - 模拟 10 个 order 的执行，每 20Hz 发布一次反馈
 * - 支持客户端取消：若收到取消则返回 canceled 结果
 * - 正常完成后返回 success=true 的结果
 *
 * 注意：此处为示例/占位实现。真实系统应：
 * 1) 解析 goal->plan_json
 * 2) 按照计划调度底层控制器（路径跟随、速度/姿态控制等）
 * 3) 将实时状态转换为反馈并发布
 */
void MotionControlCenter::execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExecutePlan::Feedback>();
  auto result = std::make_shared<ExecutePlan::Result>();

  // 初始反馈：空闲状态
  feedback->current_order = 0;
  feedback->status = ExecutePlan::Feedback::STATUS_IDLE;
  goal_handle->publish_feedback(feedback);

  // 模拟执行：共 10 个步骤（order），每个步骤发布一次反馈
  const uint32_t total_orders = 10;  // 示例占位

  rclcpp::Rate loop_rate(20);  // 20Hz 发布反馈
  for (uint32_t order = 1; order <= total_orders; ++order) {
    // 支持取消：收到取消则返回 canceled
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->error_message = "客户端取消执行";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "目标已取消：plan_uid=%s", goal->plan_uid.c_str());
      return;
    }

    feedback->current_order = order;
    feedback->status = ExecutePlan::Feedback::STATUS_EXECUTING;
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  // 执行完成：返回成功
  result->success = true;
  result->error_message = "";
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(), "目标执行完成：plan_uid=%s", goal->plan_uid.c_str());
}

}  // namespace base_controller
}  // namespace xline





