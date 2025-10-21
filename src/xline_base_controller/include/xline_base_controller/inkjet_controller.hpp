#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <xline_msgs/srv/quick_command.hpp>
#include <xline_msgs/srv/configure_print.hpp>

namespace xline
{
  namespace base_controller
  {

    /**
     * 喷墨控制器配置结构体
     * 包含喷墨打印的所有配置参数
     */
    struct InkConfig
    {
      bool enabled = false;                     // 是否启用喷墨
      std::string mode;                         // 打印模式: "solid"(实线), "dashed"(虚线), "text"(文字)
      std::vector<std::string> printers;        // 打印机列表: ["left", "center", "right"]

      // 通用参数
      int start_delay_ms = 0;                   // 开始喷墨前延时(毫秒)
      int stop_delay_ms = 0;                    // 停止喷墨前延时(毫秒)

      // 虚线参数
      double dash_length_mm = 50.0;             // 实线段长度(毫米)
      double gap_length_mm = 30.0;              // 间隔长度(毫米)
      bool start_with_dash = true;              // 是否以实线开始

      // 文字参数
      std::string text_content;                 // 要打印的文字内容
      int font_size = 12;                       // 字体大小
      bool repeat = false;                      // 是否重复打印
    };

    /**
     * 喷墨控制器类
     * 负责管理喷墨打印机的控制逻辑，支持实线、虚线、文字三种打印模式
     * 内部自动创建和管理所需的服务客户端
     */
    class InkjetController
    {
    public:
      /**
       * 构造函数
       * @param node ROS 2 节点指针（用于创建服务客户端）
       */
      explicit InkjetController(rclcpp::Node* node);

      /**
       * 重置控制器状态
       * 清理所有内部状态，为下一次使用做准备
       * 注意：不会销毁服务客户端，只是重置状态变量
       */
      void reset();

      /**
       * 初始化喷墨控制器
       * 对于文字模式，会提前配置文字内容到打印机
       * @param config 喷墨配置参数
       * @return 初始化是否成功
       */
      bool initialize(const InkConfig& config);

      /**
       * 开始打印
       * 向所有配置的打印机发送 start_print 命令
       * @return 是否成功开始打印
       */
      bool startPrint();

      /**
       * 停止打印
       * 向所有配置的打印机发送 stop_print 命令
       * @return 是否成功停止打印
       */
      bool stopPrint();

      /**
       * 更新喷墨状态(主要用于虚线模式)
       * 根据已行驶距离自动开关喷墨
       * @param traveled_distance_mm 已行驶距离(毫米)
       */
      void update(double traveled_distance_mm);

      /**
       * 清理资源
       * 确保停止所有打印操作
       */
      void cleanup();

      /**
       * 获取当前是否正在打印
       * @return true 表示正在打印
       */
      bool isPrinting() const { return is_printing_; }

    private:
      // ROS 2 节点和服务客户端
      rclcpp::Node* node_;
      rclcpp::Client<xline_msgs::srv::QuickCommand>::SharedPtr quick_client_;
      rclcpp::Client<xline_msgs::srv::ConfigurePrint>::SharedPtr config_client_;

      // 配置和状态
      InkConfig config_;
      bool is_printing_ = false;                // 当前是否正在打印
      double last_switch_distance_ = 0.0;       // 上次切换状态时的距离(用于虚线模式)

      /**
       * 控制喷墨打印机
       * @param action 动作类型: "start_print" 或 "stop_print"
       * @param printer_name 打印机名称
       * @return 是否执行成功
       */
      bool controlInkjet(const std::string& action, const std::string& printer_name);

      /**
       * 配置文字内容到打印机
       * @return 是否配置成功
       */
      bool configureText();
    };

  } // namespace base_controller
} // namespace xline
