#include "xline_base_controller/inkjet_controller.hpp"

namespace xline
{
  namespace base_controller
  {

    InkjetController::InkjetController(rclcpp::Node* node)
        : node_(node)
    {
      // 创建 QuickCommand 服务客户端（用于 start_print/stop_print）
      quick_client_ = node_->create_client<xline_msgs::srv::QuickCommand>(
          "printer/quick_command");

      // 创建 ConfigurePrint 服务客户端（用于配置文字内容）
      config_client_ = node_->create_client<xline_msgs::srv::ConfigurePrint>(
          "printer/configure_print");

      RCLCPP_DEBUG(node_->get_logger(), "InkjetController 已创建服务客户端");
    }

    void InkjetController::reset()
    {
      // 确保停止所有打印操作
      if (is_printing_)
      {
        RCLCPP_DEBUG(node_->get_logger(), "重置前停止打印");
        stopPrint();
      }

      // 重置所有状态标志
      is_printing_ = false;
      last_switch_distance_ = 0.0;

      // 清空配置（重置为默认值）
      config_ = InkConfig();

      RCLCPP_DEBUG(node_->get_logger(), "InkjetController 状态已重置");
    }

    bool InkjetController::initialize(const InkConfig& config)
    {
      config_ = config;

      if (!config_.enabled)
      {
        RCLCPP_DEBUG(node_->get_logger(), "喷墨功能未启用");
        return true;
      }

      RCLCPP_INFO(node_->get_logger(), "初始化喷墨控制器 - 模式: %s, 打印机数量: %zu",
                  config_.mode.c_str(), config_.printers.size());

      // 文字模式：提前配置文字内容
      if (config_.mode == "text")
      {
        RCLCPP_INFO(node_->get_logger(), "配置文字内容: %s", config_.text_content.c_str());
        return configureText();
      }

      // 虚线模式：输出虚线参数
      if (config_.mode == "dashed")
      {
        RCLCPP_INFO(node_->get_logger(),
                    "虚线参数 - 实线段: %.1fmm, 间隔: %.1fmm, 起始: %s",
                    config_.dash_length_mm,
                    config_.gap_length_mm,
                    config_.start_with_dash ? "实线" : "间隔");
      }

      return true;
    }

    bool InkjetController::startPrint()
    {
      // 检查是否启用喷墨
      if (!config_.enabled)
      {
        RCLCPP_DEBUG(node_->get_logger(), "喷墨未启用，跳过开始打印");
        return true;
      }

      // 检查是否已经在打印中（避免重复调用）
      if (is_printing_)
      {
        RCLCPP_DEBUG(node_->get_logger(), "已经在打印中，跳过重复的开始打印命令");
        return true;
      }

      // 可选延时
      if (config_.start_delay_ms > 0)
      {
        RCLCPP_DEBUG(node_->get_logger(), "开始喷墨前延时 %d ms", config_.start_delay_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.start_delay_ms));
      }

      bool success = true;
      for (const auto& printer : config_.printers)
      {
        RCLCPP_INFO(node_->get_logger(), "开始喷墨: %s (模式: %s)",
                    printer.c_str(), config_.mode.c_str());
        success &= controlInkjet("start_print", printer);
      }

      if (success)
      {
        is_printing_ = true;
        last_switch_distance_ = 0.0;
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "部分打印机启动失败");
      }

      return success;
    }

    bool InkjetController::stopPrint()
    {
      // 检查是否启用喷墨
      if (!config_.enabled)
      {
        RCLCPP_DEBUG(node_->get_logger(), "喷墨未启用，跳过停止打印");
        return true;
      }

      // 检查是否已经停止（避免重复调用）
      if (!is_printing_)
      {
        RCLCPP_DEBUG(node_->get_logger(), "已经停止打印，跳过重复的停止打印命令");
        return true;
      }

      // 可选延时
      if (config_.stop_delay_ms > 0)
      {
        RCLCPP_DEBUG(node_->get_logger(), "停止喷墨前延时 %d ms", config_.stop_delay_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.stop_delay_ms));
      }

      bool success = true;
      for (const auto& printer : config_.printers)
      {
        RCLCPP_INFO(node_->get_logger(), "停止喷墨: %s", printer.c_str());
        success &= controlInkjet("stop_print", printer);
      }

      is_printing_ = false;

      if (!success)
      {
        RCLCPP_WARN(node_->get_logger(), "部分打印机停止失败");
      }

      return success;
    }

    void InkjetController::update(double traveled_distance_mm)
    {
      // 仅虚线模式需要更新
      if (!config_.enabled || config_.mode != "dashed")
      {
        return;
      }

      // 计算当前在虚线周期中的位置
      double cycle_length = config_.dash_length_mm + config_.gap_length_mm;
      double position_in_cycle = fmod(traveled_distance_mm, cycle_length);

      // 判断当前应该打印还是不打印
      bool should_print = (position_in_cycle < config_.dash_length_mm);

      // 如果不是以实线开始，则反转状态
      if (!config_.start_with_dash)
      {
        should_print = !should_print;
      }

      // 状态切换
      if (should_print && !is_printing_)
      {
        // 需要开启打印
        RCLCPP_DEBUG(node_->get_logger(), "虚线-开启打印 (距离: %.1fmm, 周期位置: %.1fmm)",
                     traveled_distance_mm, position_in_cycle);

        for (const auto& printer : config_.printers)
        {
          controlInkjet("start_print", printer);
        }
        is_printing_ = true;
        last_switch_distance_ = traveled_distance_mm;
      }
      else if (!should_print && is_printing_)
      {
        // 需要关闭打印
        RCLCPP_DEBUG(node_->get_logger(), "虚线-关闭打印 (距离: %.1fmm, 周期位置: %.1fmm)",
                     traveled_distance_mm, position_in_cycle);

        for (const auto& printer : config_.printers)
        {
          controlInkjet("stop_print", printer);
        }
        is_printing_ = false;
        last_switch_distance_ = traveled_distance_mm;
      }
    }

    void InkjetController::cleanup()
    {
      // 停止打印（stopPrint() 内部已经有状态检查，无需重复检查）
      stopPrint();

      // 未来可以在这里添加其他清理逻辑：
      // - 清理缓存
      // - 重置某些标志
      // - 断开连接等
    }

    bool InkjetController::controlInkjet(const std::string& action, const std::string& printer_name)
    {
      if (!quick_client_)
      {
        RCLCPP_ERROR(node_->get_logger(), "QuickCommand 服务客户端未初始化");
        return false;
      }

      if (!quick_client_->service_is_ready())
      {
        RCLCPP_WARN(node_->get_logger(), "QuickCommand 服务不可用");
        return false;
      }

      auto request = std::make_shared<xline_msgs::srv::QuickCommand::Request>();
      request->printer_name = printer_name;
      request->action = action;
      request->param = 0;

      auto future = quick_client_->async_send_request(request);

      // 等待服务响应(最多3秒)
      auto status = future.wait_for(std::chrono::seconds(3));
      if (status == std::future_status::ready)
      {
        auto response = future.get();
        if (!response->success)
        {
          RCLCPP_WARN(node_->get_logger(), "喷墨控制失败 [%s, %s]: %s",
                      printer_name.c_str(), action.c_str(), response->message.c_str());
        }
        return response->success;
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "QuickCommand 服务调用超时 [%s, %s]",
                     printer_name.c_str(), action.c_str());
        return false;
      }
    }

    bool InkjetController::configureText()
    {
      if (!config_client_)
      {
        RCLCPP_ERROR(node_->get_logger(), "ConfigurePrint 服务客户端未初始化");
        return false;
      }

      if (!config_client_->service_is_ready())
      {
        RCLCPP_WARN(node_->get_logger(), "ConfigurePrint 服务不可用");
        return false;
      }

      bool success = true;
      for (const auto& printer : config_.printers)
      {
        auto request = std::make_shared<xline_msgs::srv::ConfigurePrint::Request>();
        request->printer_name = printer;
        request->content = config_.text_content;
        request->font_size = config_.font_size;
        request->repeat = config_.repeat;

        auto future = config_client_->async_send_request(request);

        // 等待服务响应(最多3秒)
        auto status = future.wait_for(std::chrono::seconds(3));
        if (status == std::future_status::ready)
        {
          auto response = future.get();
          RCLCPP_INFO(node_->get_logger(), "文字配置 [%s]: %s",
                      printer.c_str(), response->message.c_str());
          success &= response->success;
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "ConfigurePrint 服务调用超时: %s", printer.c_str());
          success = false;
        }
      }

      return success;
    }

  } // namespace base_controller
} // namespace xline
