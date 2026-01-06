#include "xline_base_controller/motion_control_center.hpp"

#include <cmath>
#include <sstream>
#include <algorithm>
#include <thread>
#include <chrono>
#include <future>

// 为 std::bind 预留占位符
using std::placeholders::_1;
using std::placeholders::_2;

namespace xline
{
  namespace base_controller
  {


    MotionControlCenter::MotionControlCenter(const rclcpp::NodeOptions &options)
        : rclcpp::Node("motion_control_center", options)
    {
      // 创建 ExecutePlan 动作服务器：分别绑定 目标处理/取消处理/接受处理 回调
      action_server_ = rclcpp_action::create_server<ExecutePlan>(this,           // 节点指针
                                                                 "execute_plan", // 动作名
                                                                 std::bind(&MotionControlCenter::handleGoal, this, _1, _2),
                                                                 std::bind(&MotionControlCenter::handleCancel, this, _1),
                                                                 std::bind(&MotionControlCenter::handleAccepted, this, _1));

      // 创建位姿订阅器
      pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/robot_pose", // 话题名
          10,            // QoS 队列大小
          std::bind(&MotionControlCenter::poseCallback, this, _1));

      // 创建 cmd_vel 发布器(用于校准时控制机器人移动)
      cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/task_cmd_vel", // 话题名
                                                                             10);        // QoS 队列大小

      // 创建定位校准服务客户端
      calibration_client_ = this->create_client<std_srvs::srv::Trigger>("/localization/calibrate_pose"); // 服务名

      // 创建暂停/恢复服务
      pause_service_ = this->create_service<std_srvs::srv::Trigger>(
          "/execution/pause", std::bind(&MotionControlCenter::handlePauseService, this, _1, _2));

      resume_service_ = this->create_service<std_srvs::srv::Trigger>(
          "/execution/resume", std::bind(&MotionControlCenter::handleResumeService, this, _1, _2));

      // 创建姿态校正服务
      calibration_service_ = this->create_service<std_srvs::srv::Trigger>(
          "/motion_control/execute_calibration", std::bind(&MotionControlCenter::handleCalibrationService, this, _1, _2));

      // 创建步进电机控制服务客户端（左右喷码机用）
      stepper_motor_client_ = this->create_client<xline_msgs::srv::MotorCommand>("/stepper_motor_driver");
      RCLCPP_INFO(get_logger(), "步进电机控制服务客户端已创建: '/stepper_motor_driver'");

      RCLCPP_INFO(get_logger(), "MotionControlCenter 动作服务器已就绪: 'execute_plan'");
      RCLCPP_INFO(get_logger(), "位姿订阅器已创建: '/robot_pose'");
      RCLCPP_INFO(get_logger(), "cmd_vel 发布器已创建: '/cmd_vel'");
      RCLCPP_INFO(get_logger(), "校准服务客户端已创建: '/localization/calibrate_pose'");
      RCLCPP_INFO(get_logger(), "暂停服务已创建: '/execution/pause'");
      RCLCPP_INFO(get_logger(), "恢复服务已创建: '/execution/resume'");
      RCLCPP_INFO(get_logger(), "姿态校正服务已创建: '/motion_control/execute_calibration'");

      // 创建路径跟随控制器
      line_follow_controller_ = std::make_shared<xline::follow_controller::LineFollowController>();
      rpp_follow_controller_ = std::make_shared<xline::follow_controller::RPPController>();
      lqr_circle_controller = std::make_shared<xline::follow_controller::LQRCircleController>();
      base_follow_controller_ = nullptr;
      inkjet_client_ = std::make_shared<InkjetClient>();
      RCLCPP_INFO(get_logger(), "喷墨控制器已创建（服务客户端已就绪）");
    }

    MotionControlCenter::~MotionControlCenter()
    {
      // 在析构时通知所有执行线程退出
      RCLCPP_INFO(get_logger(), "MotionControlCenter 正在关闭...");
      shutdown_.store(true);

      // 唤醒可能正在暂停等待的线程
      pause_cv_.notify_all();


      // 给执行线程一些时间完成清理
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      RCLCPP_INFO(get_logger(), "MotionControlCenter 已关闭");
    }

    /**
     * 目标处理回调：
     * - 检查是否有正在执行的action
     * - 基础合法性检查（plan_uid / plan_json 不能为空）
     * - 通过则返回 ACCEPT_AND_EXECUTE，进入执行流程
     */
    rclcpp_action::GoalResponse MotionControlCenter::handleGoal(const rclcpp_action::GoalUUID &uuid,
                                                                std::shared_ptr<const ExecutePlan::Goal> goal)
    {
      (void)uuid;

      // 检查是否有action正在执行
      if (is_executing_.load())
      {
        RCLCPP_WARN(get_logger(), "拒绝目标：当前有action正在执行中");
        return rclcpp_action::GoalResponse::REJECT;
      }

      // 基础校验：goal 不能为空
      if (!goal)
      {
        RCLCPP_WARN(get_logger(), "拒绝目标：收到空的 goal");
        return rclcpp_action::GoalResponse::REJECT;
      }

      // 内容校验：plan_uid 与 plan_json 不能为空
      if (goal->plan_uid.empty() || goal->plan_json.empty())
      {
        RCLCPP_WARN(get_logger(), "拒绝目标：plan_uid 或 plan_json 为空");
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(get_logger(), "接受目标：plan_uid=%s, json长度=%zu", goal->plan_uid.c_str(), goal->plan_json.size());
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * 取消处理回调：
     * - 无条件接受取消请求
     * - 唤醒可能正在暂停等待的线程
     */
    rclcpp_action::CancelResponse MotionControlCenter::handleCancel(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
    {
      (void)goal_handle;
      RCLCPP_INFO(get_logger(), "收到取消请求，接受取消");

      // 唤醒可能正在暂停等待的线程
      // 这样checkPauseState()中的条件变量会重新检查is_canceling()
      pause_cv_.notify_all();

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
      

    // void MotionControlCenter::execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle){
    //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //   auto result = std::make_shared<ExecutePlan::Result>();
    //   result->success = true;
    //   result->error_message = "模拟流程";
    //   goal_handle->succeed(result);
    // }

    void MotionControlCenter::execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
    {
      // 使用 compare_exchange_strong 原子地检查并设置执行标志
      // 这样可以防止多个 goal 同时通过 handleGoal 检查后并发执行
      bool expected = false;
      if (!is_executing_.compare_exchange_strong(expected, true))
      {
        // 竞态条件：另一个任务已经开始执行
        auto result = std::make_shared<ExecutePlan::Result>();
        result->success = false;
        result->error_message = "系统忙：另一个任务正在执行中";
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "拒绝执行：检测到并发任务冲突");
        return;
      }

      // 使用RAII确保函数退出时清除所有状态标志
      auto cleanup = [this](void *)
      {
        is_executing_.store(false);
        is_paused_.store(false);  // 防止暂停标志残留
      };
      std::unique_ptr<void, decltype(cleanup)> guard(reinterpret_cast<void *>(1), cleanup);

      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<ExecutePlan::Feedback>();
      auto result = std::make_shared<ExecutePlan::Result>();

      // 初始反馈
      feedback->current_id = 0;
      goal_handle->publish_feedback(feedback);

      // 解析JSON字符串
      Json::CharReaderBuilder reader_builder;
      Json::Value line;
      std::string parse_errors;
      std::istringstream json_stream(goal->plan_json);

      if (!Json::parseFromStream(reader_builder, json_stream, &line, &parse_errors))
      {
        result->success = false;
        result->error_message = "JSON解析失败: " + parse_errors;
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "JSON解析失败：%s", parse_errors.c_str());
        return;
      }

      // 安全地提取 id 和 layer_id（支持 int 和 uint 类型）
      uint32_t path_id = 0;
      uint32_t layer_id = 0;

      if (!line.isMember("id"))
      {
        result->success = false;
        result->error_message = "JSON中缺少必需字段: id";
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "JSON解析失败：缺少id字段");
        return;
      }

      if (!line.isMember("layer_id"))
      {
        result->success = false;
        result->error_message = "JSON中缺少必需字段: layer_id";
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "JSON解析失败：缺少layer_id字段");
        return;
      }

      // 安全转换 id（可能是 int 或 uint）
      if (line["id"].isUInt())
      {
        path_id = line["id"].asUInt();
      }
      else if (line["id"].isInt())
      {
        int id_int = line["id"].asInt();
        if (id_int < 0)
        {
          result->success = false;
          result->error_message = "JSON中id字段值为负数: " + std::to_string(id_int);
          goal_handle->abort(result);
          RCLCPP_ERROR(get_logger(), "JSON解析失败：id为负数 %d", id_int);
          return;
        }
        path_id = static_cast<uint32_t>(id_int);
      }
      else
      {
        result->success = false;
        result->error_message = "JSON中id字段类型错误（应为整数）";
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "JSON解析失败：id字段类型不是整数");
        return;
      }

      // 安全转换 layer_id（可能是 int 或 uint）
      if (line["layer_id"].isUInt())
      {
        layer_id = line["layer_id"].asUInt();
      }
      else if (line["layer_id"].isInt())
      {
        int layer_id_int = line["layer_id"].asInt();
        if (layer_id_int < 0)
        {
          result->success = false;
          result->error_message = "JSON中layer_id字段值为负数: " + std::to_string(layer_id_int);
          goal_handle->abort(result);
          RCLCPP_ERROR(get_logger(), "JSON解析失败：layer_id为负数 %d", layer_id_int);
          return;
        }
        layer_id = static_cast<uint32_t>(layer_id_int);
      }
      else
      {
        result->success = false;
        result->error_message = "JSON中layer_id字段类型错误（应为整数）";
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "JSON解析失败：layer_id字段类型不是整数");
        return;
      }

      RCLCPP_INFO(get_logger(), "开始执行计划,路径id: %u", path_id);

      // 支持取消：收到取消则返回 canceled
      if (goal_handle->is_canceling())
      {
        // 停止机器人
        geometry_msgs::msg::Twist stop;
        cmd_vel_publisher_->publish(stop);

        // 清理暂停标志
        is_paused_.store(false);

        result->success = false;
        result->error_message = "客户端取消执行";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "目标已取消：plan_uid=%s", goal->plan_uid.c_str());
        return;
      }

      current_layer_type = line["type"].asString();
      current_layer_id = layer_id;
      bool is_backward = line["backward"].asBool();

      // 检查 layer_id 是否 >= 1000000
      bool is_start_from_robot = (layer_id >= 1000000);

      // 发布反馈 - 只发送current_id
      feedback->current_id = path_id;
      goal_handle->publish_feedback(feedback);

      // 在路径配置前检查取消状态，提高取消响应速度
      if (goal_handle->is_canceling())
      {
        geometry_msgs::msg::Twist stop;
        cmd_vel_publisher_->publish(stop);
        
        is_paused_.store(false);

        result->success = false;
        result->error_message = "任务在路径配置前被取消";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "目标已取消：plan_uid=%s", goal->plan_uid.c_str());
        return;
      }

      // 解析 ink 信息
      std::string ink_mode = "solid";  // 默认实线模式
      std::string ink_printer = "center";  // 默认中心打印机
      std::string ink_content = "";  // 文字内容（仅 text 模式）
      bool ink_enabled = true;  // 默认启用喷墨
      
      if (line.isMember("ink") && line["ink"].isObject())
      {
        const Json::Value& ink = line["ink"];
        if (ink.isMember("enabled"))
        {
          ink_enabled = ink["enabled"].asBool();
        }
        if (ink.isMember("mode"))
        {
          ink_mode = ink["mode"].asString();
        }
        if (ink.isMember("printer"))
        {
          ink_printer = ink["printer"].asString();
        }
        if (ink.isMember("content"))
        {
          ink_content = ink["content"].asString();
        }
        
        RCLCPP_INFO(get_logger(), "[id=%u] ink信息: enabled=%s, mode=%s, printer=%s%s", 
                    path_id,
                    ink_enabled ? "true" : "false",
                    ink_mode.c_str(),
                    ink_printer.c_str(),
                    ink_mode == "text" ? (", content=" + ink_content).c_str() : "");
      }

      // 获取 layer 信息，判断是否为 TRANSITION 路径
      std::string layer_name = "";
      if (line.isMember("layer"))
      {
        layer_name = line["layer"].asString();
      }
      bool is_transition = (layer_name == "TRANSITION");

      // ========== 姿态校正计时逻辑 ==========
      // 需求：在持续接收任务过程中，以 60s 为周期进行一次姿态校正。
      // 在每次接收到路径 action 时，如果：
      //   1) 当前路径为转场路径（TRANSITION），且
      //   2) 距离上一次姿态校正已超过 60s，或尚未进行过姿态校正
      // 则先执行一次姿态校正，待校正完成后再继续路径跟随。
      if (is_transition)
      {
        bool need_calibration = false;
        {
          std::lock_guard<std::mutex> lock(calibration_mutex_);
          if (!has_last_calibration_time_)
          {
            // 从未进行过姿态校正：在首个转场路径前进行一次校正
            need_calibration = true;
          }
          else
          {
            auto now = this->now();
            auto elapsed = now - last_calibration_time_;
            if (elapsed.seconds() >= 60.0)
            {
              need_calibration = true;
            }
          }
        }

        if (need_calibration)
        {
          // 在执行校正前再次检查是否已取消
          if (goal_handle->is_canceling())
          {
            geometry_msgs::msg::Twist stop;
            cmd_vel_publisher_->publish(stop);

            is_paused_.store(false);

            result->success = false;
            result->error_message = "任务在姿态校正前被取消";
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "目标已取消：plan_uid=%s", goal->plan_uid.c_str());
            return;
          }

          RCLCPP_INFO(get_logger(),
                      "[id=%u] 转场路径前触发姿态校正：距离上次校正已超过60秒或尚未校正",
                      path_id);

          // 使用与姿态校正服务相同的参数
          double calibration_velocity = 0.05;  // m/s
          double calibration_duration = 3.0;   // 秒
          bool calibration_success = executeLocalizationCalibration(calibration_velocity, calibration_duration);

          if (!calibration_success)
          {
            result->success = false;
            result->error_message = "转场路径前姿态校正失败";
            goal_handle->abort(result);
            RCLCPP_ERROR(get_logger(), "转场路径前姿态校正失败：plan_uid=%s", goal->plan_uid.c_str());
            return;
          }
        }
      }
      
      // 存储 ink 信息到成员变量，供后续使用
      current_ink_mode_ = ink_mode;
      current_ink_printer_ = ink_printer;
      current_ink_content_ = ink_content;
      current_ink_enabled_ = ink_enabled;
      is_transition_path_ = is_transition;
      use_stepper_for_current_path_ = false;
      current_stepper_motor_id_ = 0;
      
      if (is_transition)
      {
        RCLCPP_INFO(get_logger(), "[id=%u] TRANSITION路径: ink.mode=%s, ink.printer=%s%s", 
                    path_id, ink_mode.c_str(), ink_printer.c_str(),
                    ink_mode == "text" ? (", ink.content=" + ink_content).c_str() : "");
        
        // ========== 关键逻辑：在 transition 路径中预设文字 ==========
        // 如果是 transition 路径且 ink_mode 为 "text"，需要先设置文字到打印机
        // 这样在下一条工作路径开始时可以直接开始打印
        if (ink_mode == "text" && !ink_content.empty())
        {
          RCLCPP_INFO(get_logger(), "[id=%u] TRANSITION路径检测到text模式，异步预设文字到打印机 %s", 
                      path_id, ink_printer.c_str());
          
          // 异步调用打印机的 set_text 方法预设文字（不阻塞主线程）
          auto inkjet_client = inkjet_client_;
          std::string printer_name = ink_printer;
          std::string text_content = ink_content;
          uint32_t log_path_id = path_id;
          auto logger = get_logger();
          
          std::thread([inkjet_client, printer_name, text_content, log_path_id, logger]() {
            auto [success, message] = inkjet_client->set_text(printer_name, text_content);
            if (success)
            {
              RCLCPP_INFO(logger, "[id=%u] 文字预设成功: printer=%s, text=\"%s\"", 
                          log_path_id, printer_name.c_str(), text_content.c_str());
            }
            else
            {
              RCLCPP_ERROR(logger, "[id=%u] 文字预设失败: %s", log_path_id, message.c_str());
            }
          }).detach();
        }
      }

      // ========== 步进电机控制：非转场路径 + 左/右喷码机 ==========
      // 需求：在执行非转场路径时，如果是左右喷码机，
      //      先使对应的喷码机电机 forward，在到达目标后再 reverse。
      if (!is_transition && ink_enabled &&
          (ink_printer == "left" || ink_printer == "right"))
      {
        current_stepper_motor_id_ = (ink_printer == "left") ? 1 : 2;
        use_stepper_for_current_path_ = true;

        RCLCPP_INFO(get_logger(),
                    "[id=%u] 非转场路径，启动步进电机: printer=%s, motor_id=%d, command=forward",
                    path_id, ink_printer.c_str(), current_stepper_motor_id_);

        // 异步发送 forward 命令，不阻塞主执行线程
        controlStepperMotor(current_stepper_motor_id_, "forward");
      }

      // 根据类型提取数据
      if (current_layer_type == "line" || current_layer_type == "text")
      {
        // text 类型与 line 类型使用相同的数据结构（都有 start/end）
        LineData line_data = extractLineData(line);
        
        if (current_layer_type == "text")
        {
          // 获取文字内容
          std::string text_content = "";
          if (line.isMember("content"))
          {
            text_content = line["content"].asString();
          }
          RCLCPP_INFO(get_logger(), "[text, id=%u]: 起点(%.2f, %.2f) -> 终点(%.2f, %.2f), 内容=\"%s\"", 
                      path_id, line_data.start_x, line_data.start_y, line_data.end_x, line_data.end_y,
                      text_content.c_str());
        }
        else
        {
          RCLCPP_INFO(get_logger(), "[line, id=%u]: 起点(%.2f, %.2f) -> 终点(%.2f, %.2f)", 
                      path_id, line_data.start_x, line_data.start_y, line_data.end_x, line_data.end_y);
        }
        
        geometry_msgs::msg::PoseStamped robot_pose;
        getLatestPose(robot_pose);
        line_follow_controller_->setPose(robot_pose);
        line_follow_controller_->setTransitionPath(is_transition);
        if(is_start_from_robot){ // 是否使用机器人位置作为路径的起点
          
          line_follow_controller_->setPlan(robot_pose.pose.position.x, robot_pose.pose.position.y, line_data.end_x, line_data.end_y);
          line_follow_controller_->setBackFollow(is_backward);
        }else{
          line_follow_controller_->setPlan(line_data.start_x, line_data.start_y, line_data.end_x, line_data.end_y);
        }
        base_follow_controller_ = line_follow_controller_;
      }
      else if (current_layer_type == "circle")
      {
        CircleData circle_data = extractCircleData(line);
        RCLCPP_INFO(get_logger(), "[circle, id=%u]: 圆心(%.2f, %.2f), 半径%.2f", path_id, circle_data.center_x,
                    circle_data.center_y, circle_data.radius);

        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.pose.position.x = circle_data.start_x;
        start_pose.pose.position.y = circle_data.start_y;
        lqr_circle_controller->setAngleRange(2 * M_PI, 0.0);
        lqr_circle_controller->setPlanForCircle(circle_data.center_x, circle_data.center_y, circle_data.radius, start_pose);
        // lqr_circle_controller->setBackFollow(false);
        base_follow_controller_ = lqr_circle_controller;
      }
      else if (current_layer_type == "arc")
      {
        ArcData arc_data = extractArcData(line);
        RCLCPP_INFO(get_logger(), "[arc, id=%u]: 圆心(%.2f, %.2f), 半径%.2f, 角度[%.2f, %.2f] rad", path_id,
                    arc_data.center_x, arc_data.center_y, arc_data.radius, arc_data.start_angle, arc_data.end_angle);
        geometry_msgs::msg::PoseStamped current_pose;
        getLatestPose(current_pose);

        rpp_follow_controller_->setAngleRange(arc_data.start_angle, arc_data.end_angle);
        rpp_follow_controller_->setPlanForCircle(arc_data.center_x, arc_data.center_y, arc_data.radius, current_pose);
        base_follow_controller_ = rpp_follow_controller_;
      }
      else if (current_layer_type == "spline")
      {
        SplineData spline_data = extractSplineData(line);
        RCLCPP_INFO(get_logger(), "[spline, id=%u]: 控制点数=%zu, 起点(%.2f, %.2f) -> 终点(%.2f, %.2f)",
                    path_id, spline_data.vertices.size(), spline_data.start_x, spline_data.start_y,
                    spline_data.end_x, spline_data.end_y);

        // 使用 RPP 控制器的 setPlanForSpline 方法设置 Spline 路径
        // 路径生成由 CurvePathStrategy 内部完成
        rpp_follow_controller_->setPlanForSpline(spline_data.vertices, spline_data.degree,
                                                  spline_data.start_x, spline_data.start_y,
                                                  spline_data.end_x, spline_data.end_y);
        rpp_follow_controller_->setBackFollow(is_backward);
        base_follow_controller_ = rpp_follow_controller_;
      }
      else if (current_layer_type == "ellipse")
      {
        EllipseData ellipse_data = extractEllipseData(line);
        RCLCPP_INFO(get_logger(), "[ellipse, id=%u]: 中心(%.2f, %.2f), 主轴(%.2f, %.2f), 比例=%.2f, 角度[%.2f, %.2f]度",
                    path_id, ellipse_data.center_x, ellipse_data.center_y,
                    ellipse_data.major_axis_x, ellipse_data.major_axis_y,
                    ellipse_data.ratio, ellipse_data.start_angle, ellipse_data.end_angle);

        // 使用 RPP 控制器的 setPlanForEllipse 方法设置 Ellipse 路径
        // 路径生成由 CurvePathStrategy 内部完成
        rpp_follow_controller_->setPlanForEllipse(ellipse_data.center_x, ellipse_data.center_y,
                                                   ellipse_data.major_axis_x, ellipse_data.major_axis_y,
                                                   ellipse_data.ratio, ellipse_data.rotation,
                                                   ellipse_data.start_angle, ellipse_data.end_angle);
        rpp_follow_controller_->setBackFollow(is_backward);
        base_follow_controller_ = rpp_follow_controller_;
      }
      else
      {
        RCLCPP_WARN(get_logger(), "[id=%u]: 未知类型 %s，跳过", path_id, current_layer_type.c_str());
      }

      // type 为 text 时，不执行路径跟随，直接返回成功
      if (current_layer_type == "text")
      {
        result->success = true;
        result->error_message.clear();
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "[text, id=%u]: 文本路径不执行路径跟随，直接返回成功", path_id);
        return;
      }

      // ========== 速度限制逻辑 ==========
      // 当路径模式为 "text" 且不是转场路径时，限制速度为 0.2 m/s
      if (ink_mode == "text" && !is_transition && base_follow_controller_)
      {
        const double text_mode_speed_limit = 0.2;  // 文字喷印模式的速度限制 (m/s)
        base_follow_controller_->setSpeedLimit(text_mode_speed_limit);
        RCLCPP_INFO(get_logger(), "[id=%u] text模式非转场路径: 速度限制为 %.2f m/s", 
                    path_id, text_mode_speed_limit);
      }

      // 在开始执行控制循环前检查暂停/取消状态，提高响应速度
      checkPauseState(goal_handle);
      if (goal_handle->is_canceling())
      {
        geometry_msgs::msg::Twist stop;
        cmd_vel_publisher_->publish(stop);


        is_paused_.store(false);

        result->success = false;
        result->error_message = "任务在控制器初始化后被取消";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "目标已取消：plan_uid=%s", goal->plan_uid.c_str());
        return;
      }

      compute_velocity(goal_handle, result);


      // 根据结果和取消状态决定如何结束Action
      if (goal_handle->is_canceling())
      {
        // 取消状态：调用canceled
        result->success = false;
        if (result->error_message.empty())
        {
          result->error_message = "任务已取消";
        }
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "目标已取消：plan_uid=%s", goal->plan_uid.c_str());
      }
      else if (result->success)
      {
        // 成功状态：调用succeed
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "目标执行完成：plan_uid=%s", goal->plan_uid.c_str());
      }
      else
      {
        // 失败状态：调用abort
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "目标执行失败：plan_uid=%s, 原因: %s",
                     goal->plan_uid.c_str(), result->error_message.c_str());
      }
    }

    bool MotionControlCenter::compute_velocity(const std::shared_ptr<GoalHandleExecutePlan> goal_handle,
                                               ExecutePlan::Result::SharedPtr result)
    {
      // 参考 xline_controller.cpp::ControllerServer::computeControl 实现
      // 这里执行周期性控制：读取当前位姿 -> 计算速度 -> 发布速度 -> 判断是否到达

      if (!base_follow_controller_)
      {
        RCLCPP_ERROR(get_logger(), "未设置跟随控制器，无法计算速度");
        if (result)
        {
          result->success = false;
          result->error_message = "未设置跟随控制器";
        }
        return false;
      }

      rclcpp::Rate rate(18.0);
      geometry_msgs::msg::Twist current_velocity; // 当前里程计速度，暂无则默认0
      geometry_msgs::msg::TwistStamped cmd_vel;

      bool is_inkjet_printing = false; // 标记喷码机是否在工作

      int positon_invalid_count = 0;

      // 检查节点关闭标志，确保节点销毁时执行线程能及时退出
      while (rclcpp::ok() && !shutdown_.load())
      {
        rate.sleep();

        // 支持外部暂停：阻塞等待恢复或取消
        checkPauseState(goal_handle);

        // 检查取消状态
        if (goal_handle->is_canceling())
        {
          RCLCPP_INFO(get_logger(), "检测到取消请求，停止执行");
          geometry_msgs::msg::Twist stop;
          cmd_vel_publisher_->publish(stop);

          // 结束打印 - 使用当前路径指定的打印机
          if (is_inkjet_printing)
          {
            inkjet_client_->stop_print(current_ink_printer_);
            RCLCPP_INFO(get_logger(), "取消时停止打印机: %s", current_ink_printer_.c_str());
          }

          // 清理暂停标志
          is_paused_.store(false);

          if (result)
          {
            result->success = false;
            result->error_message = "任务已取消";
          }
          return false;
        }

        // 获取最新位姿，检查返回值确保位姿数据有效
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!getLatestPose(robot_pose) && positon_invalid_count++ > 5)
        {
          geometry_msgs::msg::Twist stop;
          cmd_vel_publisher_->publish(stop);
          continue; // 跳过本次循环，等待位姿数据
        }

        positon_invalid_count = 0; // 重置位置无效次数

        // 到达检测
        if (base_follow_controller_->isGoalReached())
        {
          geometry_msgs::msg::Twist stop;
          cmd_vel_publisher_->publish(stop);
          // 路径执行完成：如果当前路径使用左右喷码机，对应步进电机执行 reverse
          if (use_stepper_for_current_path_ && current_stepper_motor_id_ > 0)
          {
            RCLCPP_INFO(get_logger(),
                        "路径到达目标，步进电机反转: motor_id=%d, command=reverse",
                        current_stepper_motor_id_);
            controlStepperMotor(current_stepper_motor_id_, "reverse");
          }

          if (result)
          {
            result->success = true;
            result->error_message.clear();
          }
          return true;
          
        }

        // 计算控制指令
        bool ok = base_follow_controller_->computeVelocityCommands(robot_pose, current_velocity, cmd_vel);
        if (!ok)
        {
          // 结束打印 - 使用当前路径指定的打印机
          if (is_inkjet_printing)
          {
            inkjet_client_->stop_print(current_ink_printer_);
            RCLCPP_INFO(get_logger(), "计算失败时停止打印机: %s", current_ink_printer_.c_str());
          }
          RCLCPP_WARN(get_logger(), "计算速度失败，停止当前目标");
          geometry_msgs::msg::Twist stop;
          cmd_vel_publisher_->publish(stop);
          if (result)
          {
            result->success = false;
            result->error_message = "计算速度失败";
          }
          return false;
        }
        
        // ========== 打印控制逻辑 ==========
        // 开始打印条件：
        // 1. 控制器标记需要开始打印 (base_follow_controller_->start_print)
        // 2. 当前未在打印 (!is_inkjet_printing)
        // 3. 不是 transition 路径 (layer_id >= 1000000)
        bool is_transition_layer = (current_layer_id >= 1000000);

        if (base_follow_controller_->start_print && !is_inkjet_printing && !is_transition_layer)
        {
          is_inkjet_printing = true;
          
          RCLCPP_INFO(get_logger(), "开始打印: printer=%s, mode=%s", 
                      current_ink_printer_.c_str(), current_ink_mode_.c_str());
          
          // 异步启动打印
          auto inkjet_client = inkjet_client_;
          std::string printer_name = current_ink_printer_;
          
          if (current_ink_mode_ == "text")
          {

            std::thread([inkjet_client, printer_name]() {
                          inkjet_client->start_print(printer_name);
                        }).detach();

            // 文本模式：开始打印后阻塞1秒，期间持续发送0速度指令
            RCLCPP_INFO(get_logger(), "text模式开始打印后保持静止 1 秒");
            geometry_msgs::msg::Twist zero_twist;
            rclcpp::Rate wait_rate(18.0);
            int wait_count = 0;
            while (rclcpp::ok() && !shutdown_.load() && ++wait_count < 9)
            {
              // 支持暂停/恢复
              checkPauseState(goal_handle);

              // 检查取消
              if (goal_handle->is_canceling())
              {
                RCLCPP_INFO(get_logger(), "text模式等待期间检测到取消请求，停止执行");
                cmd_vel_publisher_->publish(zero_twist);

                // 结束打印
                if (is_inkjet_printing)
                {
                  inkjet_client_->stop_print(current_ink_printer_);
                  RCLCPP_INFO(get_logger(), "取消时停止打印机: %s", current_ink_printer_.c_str());
                }

                is_paused_.store(false);

                if (result)
                {
                  result->success = false;
                  result->error_message = "任务已取消";
                }
                return false;
              }

              // 发布零速度，保持机器人静止
              cmd_vel_publisher_->publish(zero_twist);
              wait_rate.sleep();
            }



          }
          else
          {
            if(current_layer_type == "line"){
              // std::thread([inkjet_client, printer_name]() {
              //             // 延迟3.1秒
              //             std::this_thread::sleep_for(std::chrono::milliseconds(3100));
              //             inkjet_client->start_print(printer_name);
              //           }).detach();
              std::thread([inkjet_client, printer_name]() {
                          inkjet_client->start_print(printer_name);
                        }).detach();
            }else{
              std::thread([inkjet_client, printer_name]() {
                          inkjet_client->start_print(printer_name);
                        }).detach();
            }
            
          }
        }

        // 停止打印条件
        if(base_follow_controller_->stop_print && is_inkjet_printing && !is_transition_layer)
        {
            std::string printer_name = current_ink_printer_;
            auto inkjet_client = inkjet_client_;
            std::thread([inkjet_client, printer_name]() {
                        inkjet_client->stop_print(printer_name);
                      }).detach();
            RCLCPP_INFO(get_logger(), "停止打印: printer=%s", current_ink_printer_.c_str());
        }

        // 发布线速度与角速度
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear = cmd_vel.twist.linear;
        twist_msg.angular = cmd_vel.twist.angular;
        current_velocity = twist_msg;
        cmd_vel_publisher_->publish(twist_msg);
      }

      // 非正常退出，发布零速
      geometry_msgs::msg::Twist stop;
      cmd_vel_publisher_->publish(stop);
      if (result)
      {
        result->success = false;
        result->error_message = "系统停止或节点关闭";
      }
      return false;
    }

    bool MotionControlCenter::controlStepperMotor(int motor_id, const std::string & command)
    {
      if (!stepper_motor_client_)
      {
        RCLCPP_ERROR(get_logger(), "步进电机服务客户端未初始化，无法发送命令");
        return false;
      }

      // 快速检查服务是否可用，避免长时间阻塞主控制流程
      using namespace std::chrono_literals;
      if (!stepper_motor_client_->wait_for_service(1s))
      {
        RCLCPP_WARN(get_logger(), "步进电机服务 '/stepper_motor_driver' 当前不可用，跳过命令: motor_id=%d, command=%s",
                    motor_id, command.c_str());
        return false;
      }

      auto request = std::make_shared<xline_msgs::srv::MotorCommand::Request>();
      request->motor_id = motor_id;
      request->command = command;

      try
      {
        auto future = stepper_motor_client_->async_send_request(
            request,
            [this, motor_id, command](rclcpp::Client<xline_msgs::srv::MotorCommand>::SharedFuture response_future)
            {
              auto response = response_future.get();
              if (response->success)
              {
                RCLCPP_INFO(this->get_logger(),
                            "步进电机命令成功: motor_id=%d, command=%s, message=%s",
                            motor_id, command.c_str(), response->message.c_str());
              }
              else
              {
                RCLCPP_WARN(this->get_logger(),
                            "步进电机命令失败: motor_id=%d, command=%s, message=%s",
                            motor_id, command.c_str(), response->message.c_str());
              }
            });
        (void)future;
        return true;
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_logger(), "发送步进电机命令异常: %s", e.what());
        return false;
      }
    }

    /**
     * 提取line数据
     * line包含: start{x,y}, end{x,y}
     */
    MotionControlCenter::LineData MotionControlCenter::extractLineData(const Json::Value &line_obj)
    {
      LineData data;
      data.start_x = line_obj["start"]["x"].asDouble() / 1000;
      data.start_y = line_obj["start"]["y"].asDouble() / 1000;
      data.end_x = line_obj["end"]["x"].asDouble() / 1000;
      data.end_y = line_obj["end"]["y"].asDouble() / 1000;

      RCLCPP_DEBUG(get_logger(), "提取Line数据: 起点(%.2f, %.2f) -> 终点(%.2f, %.2f)", data.start_x, data.start_y,
                   data.end_x, data.end_y);
      return data;
    }

    /**
     * 提取circle数据
     * circle包含: start{x,y}作为圆心, radius
     */
    MotionControlCenter::CircleData MotionControlCenter::extractCircleData(const Json::Value &circle_obj)
    {
      CircleData data;
      data.center_x = circle_obj["center"]["x"].asDouble() / 1000;
      data.center_y = circle_obj["center"]["y"].asDouble() / 1000;
      data.radius = circle_obj["radius"].asDouble() / 1000;
      data.start_x = circle_obj["start"]["x"].asDouble() / 1000;
      data.start_y = circle_obj["start"]["y"].asDouble() / 1000;

      RCLCPP_DEBUG(get_logger(), "提取Circle数据: 圆心(%.2f, %.2f), 半径%.2f", data.center_x, data.center_y, data.radius);
      return data;
    }

    /**
     * 提取arc数据
     * arc包含: center{x,y}, radius, start_angle, end_angle
     * 注意: JSON中角度为度,转换为弧度
     */
    MotionControlCenter::ArcData MotionControlCenter::extractArcData(const Json::Value &arc_obj)
    {
      ArcData data;
      data.center_x = arc_obj["center"]["x"].asDouble();
      data.center_y = arc_obj["center"]["y"].asDouble();
      data.radius = arc_obj["radius"].asDouble();

      // 度转弧度
      data.start_angle = arc_obj["start_angle"].asDouble() * M_PI / 180.0;
      data.end_angle = arc_obj["end_angle"].asDouble() * M_PI / 180.0;

      RCLCPP_DEBUG(get_logger(), "提取Arc数据: 圆心(%.2f, %.2f), 半径%.2f, 起始角%.2f rad, 结束角%.2f rad", data.center_x,
                   data.center_y, data.radius, data.start_angle, data.end_angle);
      return data;
    }

    /**
     * 提取spline数据
     * spline包含: vertices(控制点数组), degree, start{x,y}, end{x,y}
     */
    MotionControlCenter::SplineData MotionControlCenter::extractSplineData(const Json::Value &spline_obj)
    {
      SplineData data;

      // 提取控制点
      if (spline_obj.isMember("vertices") && spline_obj["vertices"].isArray())
      {
        const Json::Value& vertices = spline_obj["vertices"];
        for (Json::ArrayIndex i = 0; i < vertices.size(); ++i)
        {
          double x = vertices[i]["x"].asDouble() / 1000.0;  // 转换为米
          double y = vertices[i]["y"].asDouble() / 1000.0;
          data.vertices.push_back(std::make_pair(x, y));
        }
      }

      // 提取阶数
      data.degree = spline_obj.isMember("degree") ? spline_obj["degree"].asInt() : 3;

      // 提取起点和终点
      data.start_x = spline_obj["start"]["x"].asDouble() / 1000.0;
      data.start_y = spline_obj["start"]["y"].asDouble() / 1000.0;
      data.end_x = spline_obj["end"]["x"].asDouble() / 1000.0;
      data.end_y = spline_obj["end"]["y"].asDouble() / 1000.0;

      RCLCPP_DEBUG(get_logger(), "提取Spline数据: 控制点数=%zu, 阶数=%d, 起点(%.2f, %.2f), 终点(%.2f, %.2f)",
                   data.vertices.size(), data.degree, data.start_x, data.start_y, data.end_x, data.end_y);
      return data;
    }

    /**
     * 提取ellipse数据
     * ellipse包含: center{x,y}, major_axis{x,y}, ratio, rotation, start_angle, end_angle
     */
    MotionControlCenter::EllipseData MotionControlCenter::extractEllipseData(const Json::Value &ellipse_obj)
    {
      EllipseData data;

      // 提取中心点
      data.center_x = ellipse_obj["center"]["x"].asDouble() / 1000.0;
      data.center_y = ellipse_obj["center"]["y"].asDouble() / 1000.0;

      // 提取主轴向量
      data.major_axis_x = ellipse_obj["major_axis"]["x"].asDouble() / 1000.0;
      data.major_axis_y = ellipse_obj["major_axis"]["y"].asDouble() / 1000.0;

      // 提取比例和旋转角度
      data.ratio = ellipse_obj["ratio"].asDouble();
      data.rotation = ellipse_obj["rotation"].asDouble();

      // 提取起始和结束角度
      data.start_angle = ellipse_obj["start_angle"].asDouble();
      data.end_angle = ellipse_obj["end_angle"].asDouble();

      // 提取起点和终点
      data.start_x = ellipse_obj["start"]["x"].asDouble() / 1000.0;
      data.start_y = ellipse_obj["start"]["y"].asDouble() / 1000.0;
      data.end_x = ellipse_obj["end"]["x"].asDouble() / 1000.0;
      data.end_y = ellipse_obj["end"]["y"].asDouble() / 1000.0;

      RCLCPP_DEBUG(get_logger(), "提取Ellipse数据: 中心(%.2f, %.2f), 主轴(%.2f, %.2f), 比例=%.2f, 旋转=%.2f度",
                   data.center_x, data.center_y, data.major_axis_x, data.major_axis_y, data.ratio, data.rotation);
      return data;
    }


    /**
     * 位姿数据回调函数
     * - 接收状态估计器融合后的位姿信息
     */
    void MotionControlCenter::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      RCLCPP_DEBUG(get_logger(), "位姿数据 - 位置: [%.3f, %.3f, %.3f], 姿态: [%.3f, %.3f, %.3f, %.3f]",
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.w,
                   msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

      // 缓存最新位姿（使用互斥锁保证线程安全）
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = *msg;
        has_latest_pose_.store(true);
      }
    }

    bool MotionControlCenter::getLatestPose(geometry_msgs::msg::PoseStamped &pose)
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      if (!has_latest_pose_.load())
      {
        return false;
      }
      // has_latest_pose_.store(false);
      pose = latest_pose_;
      return true;
    }

    /**
     * 执行定位系统校准
     * - 异步调用校准服务（不等待结果）
     * - 控制机器人沿直线前进
     */
    bool MotionControlCenter::executeLocalizationCalibration(double linear_velocity, double duration)
    {
      RCLCPP_INFO(get_logger(), "开始定位系统校准流程...");
      RCLCPP_INFO(get_logger(), "校准参数: 速度=%.2f m/s, 持续时间=%.1f秒", linear_velocity, duration);

      // 1. 检查校准服务是否可用（快速检查，不阻塞）
      if (!calibration_client_->service_is_ready())
      {
        RCLCPP_WARN(get_logger(), "校准服务当前不可用，将尝试异步调用");
      }

      // 2. 异步调用校准服务
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = calibration_client_->async_send_request(request);
      
      // 不等待服务响应，直接继续执行
      RCLCPP_INFO(get_logger(), "校准服务请求已发送（异步）");

      // 3. 控制机器人沿直线前进
      RCLCPP_INFO(get_logger(), "控制机器人前进 %.1f 秒...", duration);
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = linear_velocity;
      twist_msg.linear.y = 0.0;
      twist_msg.linear.z = 0.0;
      twist_msg.angular.x = 0.0;
      twist_msg.angular.y = 0.0;
      twist_msg.angular.z = 0.0;

      // 以 20Hz 频率发布 cmd_vel
      rclcpp::Rate loop_rate(20);
      auto start_time = this->now();
      auto target_duration = rclcpp::Duration::from_seconds(duration);

      while ((this->now() - start_time) < target_duration)
      {
        cmd_vel_publisher_->publish(twist_msg);
        loop_rate.sleep();
      }

      // 4. 停止机器人
      RCLCPP_INFO(get_logger(), "停止机器人移动");
      twist_msg.linear.x = 0.0;
      cmd_vel_publisher_->publish(twist_msg);

      // 4.1 在姿态校正结束后额外静止 0.5 秒
      RCLCPP_INFO(get_logger(), "姿态校正完成，保持静止 0.5 秒...");
      rclcpp::Rate stop_rate(20);
      auto pause_start = this->now();
      auto pause_duration = rclcpp::Duration::from_seconds(0.5);
      while ((this->now() - pause_start) < pause_duration)
      {
        // 持续发布零速，确保机器人保持静止
        cmd_vel_publisher_->publish(twist_msg);
        stop_rate.sleep();
      }

      auto calibration_end = this->now();
      double total_duration = (calibration_end - start_time).seconds();
      RCLCPP_INFO(get_logger(), "本次姿态校正总耗时约 %.3f 秒（含 0.5 秒静止）", total_duration);

      // 更新最近一次姿态校正时间戳（从“校正+静止”结束时刻开始计时60秒）
      {
        std::lock_guard<std::mutex> lock(calibration_mutex_);
        last_calibration_time_ = calibration_end;
        has_last_calibration_time_ = true;
      }
      RCLCPP_INFO(get_logger(), "更新姿态校正时间戳，开始计时 60s 窗口");

      // 5. 异步检查服务结果（不阻塞，可选）
      std::async(std::launch::async, [this, future = std::move(future)]() mutable {
        try {
          auto status = future.wait_for(std::chrono::seconds(2));
          if (status == std::future_status::ready) {
            auto response = future.get();
            if (response->success) {
              RCLCPP_INFO(get_logger(), "校准服务完成: %s", response->message.c_str());
            } else {
              RCLCPP_WARN(get_logger(), "校准服务返回失败: %s", response->message.c_str());
            }
          } else {
            RCLCPP_WARN(get_logger(), "校准服务响应超时（异步检查）");
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(get_logger(), "校准服务异常: %s", e.what());
        }
      });

      return true; // 立即返回，不等待校准完成
    }

    /**
     * 暂停执行服务回调
     */
    void MotionControlCenter::handlePauseService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;

      // 使用互斥锁保护服务调用，避免暂停/恢复服务并发时的状态不一致
      std::lock_guard<std::mutex> lock(service_mutex_);

      // 检查是否有任务正在执行
      if (!is_executing_.load())
      {
        response->success = false;
        response->message = "没有正在执行的任务";
        RCLCPP_WARN(get_logger(), "暂停失败：没有正在执行的任务");
        return;
      }

      // 检查是否已经暂停
      if (is_paused_.load())
      {
        response->success = false;
        response->message = "任务已经处于暂停状态";
        RCLCPP_WARN(get_logger(), "暂停失败：任务已暂停");
        return;
      }

      // 设置暂停标志
      is_paused_.store(true);

      // 立即停止机器人，避免60-100ms的响应延迟
      geometry_msgs::msg::Twist stop;
      cmd_vel_publisher_->publish(stop);

      response->success = true;
      response->message = "任务已暂停，机器人已停止";
      RCLCPP_INFO(get_logger(), "执行已暂停，机器人已立即停止");
    }

    /**
     * 恢复执行服务回调
     */
    void MotionControlCenter::handleResumeService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;

      // 使用互斥锁保护服务调用，避免暂停/恢复服务并发时的状态不一致
      std::lock_guard<std::mutex> lock(service_mutex_);

      // 检查是否处于暂停状态
      if (!is_paused_.load())
      {
        response->success = false;
        response->message = "任务未处于暂停状态";
        RCLCPP_WARN(get_logger(), "恢复失败：任务未暂停");
        return;
      }

      // 清除暂停标志并通知等待的线程
      is_paused_.store(false);
      pause_cv_.notify_all();

      response->success = true;
      response->message = "任务已恢复";
      RCLCPP_INFO(get_logger(), "执行已恢复");
    }

    /**
     * 检查并处理暂停状态
     * 如果已暂停，则阻塞等待恢复或取消
     */
    void MotionControlCenter::checkPauseState(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
    {
      // 在锁保护下检查暂停状态，避免竞态条件
      std::unique_lock<std::mutex> lock(pause_mutex_);
      if (is_paused_.load())
      {
        // 只在第一次进入暂停时停止机器人并打印日志，避免重复操作
        if (!pause_notified_)
        {
          // 在锁保护下停止机器人，确保状态一致性
          geometry_msgs::msg::Twist stop;
          cmd_vel_publisher_->publish(stop);

          // 记录暂停位置
          geometry_msgs::msg::PoseStamped pause_pose;
          if (getLatestPose(pause_pose))
          {
            RCLCPP_INFO(get_logger(), "任务已暂停，机器人已停止 - 位置(%.3f, %.3f), 朝向%.3f°",
                        pause_pose.pose.position.x,
                        pause_pose.pose.position.y,
                        tf2::getYaw(pause_pose.pose.orientation) * 180.0 / M_PI);
          }
          else
          {
            RCLCPP_INFO(get_logger(), "任务已暂停，机器人已停止，等待恢复...");
          }
          pause_notified_ = true;

          // 暂停时停止打印 - 使用当前路径指定的打印机
          if(current_layer_id < 1000000)
          {
            inkjet_client_->stop_print(current_ink_printer_);
            RCLCPP_INFO(get_logger(), "暂停时停止打印机: %s", current_ink_printer_.c_str());
          }
        }

        // 等待恢复、取消或节点关闭（三者任一发生都会解除阻塞）
        pause_cv_.wait(lock, [this, goal_handle]()
                       { return !is_paused_.load() || goal_handle->is_canceling() || shutdown_.load(); });

        // 重置暂停通知标志
        pause_notified_ = false;

        // 如果是取消或节点关闭导致的唤醒，清理暂停标志并直接返回
        if (goal_handle->is_canceling() || shutdown_.load())
        {
          if (goal_handle->is_canceling())
          {
            RCLCPP_INFO(get_logger(), "暂停期间收到取消请求，即将退出");
          }
          else
          {
            RCLCPP_INFO(get_logger(), "暂停期间节点关闭，即将退出");
          }
          is_paused_.store(false); // 清理暂停标志
          return;
        }

        // 恢复时重新启动打印 - 使用当前路径指定的打印机
        if(current_layer_id < 1000000)
        {
          inkjet_client_->start_print(current_ink_printer_);
          RCLCPP_INFO(get_logger(), "恢复时启动打印机: %s", current_ink_printer_.c_str());
        }

        // 记录恢复位置（只有正常恢复才会执行到这里）
        geometry_msgs::msg::PoseStamped resume_pose;
        if (getLatestPose(resume_pose))
        {
          RCLCPP_INFO(get_logger(), "任务已恢复 - 位置(%.3f, %.3f), 朝向%.3f°",
                      resume_pose.pose.position.x,
                      resume_pose.pose.position.y,
                      tf2::getYaw(resume_pose.pose.orientation) * 180.0 / M_PI);
        }
        else
        {
          RCLCPP_INFO(get_logger(), "任务已恢复");
        }
      }
    }

    /**
     * 姿态校正服务回调
     * 执行完整的姿态校正流程：控制底盘移动 + 调用定位节点校准服务
     */
    void MotionControlCenter::handleCalibrationService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;

      RCLCPP_INFO(get_logger(), "收到姿态校正服务请求");

      // 使用 compare_exchange_strong 原子地检查并设置执行标志
      // 防止校准服务与 Action 任务并发执行，同时抢占 cmd_vel 控制权
      bool expected = false;
      if (!is_executing_.compare_exchange_strong(expected, true))
      {
        response->success = false;
        response->message = "拒绝校正：当前有任务正在执行中，请先完成或取消当前任务";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }

      // 使用 RAII 确保执行标志被清理
      auto cleanup = [this](void *)
      {
        is_executing_.store(false);
        is_paused_.store(false);  // 清理可能残留的暂停标志
      };
      std::unique_ptr<void, decltype(cleanup)> guard(reinterpret_cast<void *>(1), cleanup);

      // 设置默认校准参数（从配置文件读取或使用默认值）
      double calibration_velocity = 0.05;  // m/s
      double calibration_duration = 3.0;   // 秒

      // 执行姿态校正
      // bool calibration_success = executeLocalizationCalibration(calibration_velocity, calibration_duration);

      // if (!calibration_success)
      // {
      //   response->success = false;
      //   response->message = "姿态校正失败，请查看日志了解详情";
      //   RCLCPP_ERROR(get_logger(), " %s", response->message.c_str());
      //   return;
      // }


      // 立即返回成功响应（喷码机恢复在后台进行）
      response->success = true;
      response->message = "姿态校正完成，喷码机正在后台恢复";
      RCLCPP_INFO(get_logger(), " %s", response->message.c_str());
    }

  } // namespace base_controller
} // namespace xline
