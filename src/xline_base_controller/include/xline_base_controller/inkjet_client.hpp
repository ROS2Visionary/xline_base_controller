/**
 * @file inkjet_client_node.hpp
 * @brief 喷墨打印机 ROS 2 客户端节点
 * 
 * 一个完整的ROS 2节点，包含所有打印机服务客户端功能。
 * 
 * 使用示例 - 完整测试打印流程：
 * @code
 * #include <rclcpp/rclcpp.hpp>
 * #include "inkjet_client_node.hpp"
 * 
 * int main(int argc, char** argv) {
 *     rclcpp::init(argc, argv);
 *     
 *     auto node = std::make_shared<InkjetClient>();
 *     node->wait_for_all_services(std::chrono::seconds(5));
 *     
 *     // 执行完整的测试打印流程（一键触发）
 *     // 自动执行：设置打印模式 -> 等待1秒 -> 测试指令 -> 等待1秒 -> 开始打印
 *     auto [success, msg] = node->test_print("left");
 *     if (success) {
 *         RCLCPP_INFO(node->get_logger(), "左打印机测试流程执行成功");
 *         RCLCPP_INFO(node->get_logger(), "%s", msg.c_str());
 *     } else {
 *         RCLCPP_ERROR(node->get_logger(), "测试失败: %s", msg.c_str());
 *     }
 *     
 *     // 对所有打印机执行测试
 *     auto [success_all, msg_all] = node->test_print("all");
 *     
 *     rclcpp::shutdown();
 *     return 0;
 * }
 * @endcode
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <xline_msgs/srv/printer_command.hpp>
#include <xline_msgs/srv/quick_command.hpp>
#include <xline_msgs/srv/set_text.hpp>

#include <string>
#include <tuple>
#include <map>
#include <optional>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * @class InkjetClient
 * @brief 喷墨打印机客户端节点
 * 
 * 继承自rclcpp::Node，实现所有打印机服务的客户端功能。
 * 支持三个打印机：left、center、right
 * 
 * 主要功能：
 * - 打印控制：beep(), start_print(), stop_print()
 * - 维护操作：clean_nozzle()
 * - 测试功能：test_print() - 执行完整测试流程
 * - 状态查询：query_ink_level(), get_status_*()
 * - 打印机管理：set_printer_enabled(), set_printer_active()
 * 
 * 测试打印说明：
 *     test_print() 方法会自动执行完整的测试流程：
 *     1. 设置打印模式（interval=75ms）
 *     2. 等待1秒
 *     3. 发送测试指令（13个模块：文本+边框图形）
 *     4. 等待1秒
 *     5. 开始打印
 */

 namespace xline
{
  namespace base_controller
  {
class InkjetClient : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    InkjetClient();

    /**
     * @brief 等待所有服务可用
     * @param timeout 超时时间
     * @return 所有服务是否都可用
     */
    bool wait_for_all_services(std::chrono::seconds timeout = std::chrono::seconds(5));

    // ========== 通用命令服务方法 ==========

    /**
     * @brief 发送通用命令到指定打印机
     * @param printer_name 打印机名称 (left/center/right)
     * @param json_data JSON格式的命令数据（指令类型可由 JSON 内容推断）
     * @param timeout 超时时间
     * @return (成功标志, 消息) 元组
     */
    std::tuple<bool, std::string> send_command(
        const std::string& printer_name,
        const std::string& json_data,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    // ========== 快速命令服务方法 ==========

    /**
     * @brief 发送快速命令
     * @param printer_name 打印机名称 (left/center/right/all)
     * @param action 动作名称 (beep/start_print/stop_print/clean_nozzle/test_print/ink_level)
     * @param param 参数值（如蜂鸣次数、清洗强度）
     * @param timeout 超时时间
     * @return (成功标志, 消息) 元组
     */
    std::tuple<bool, std::string> quick_command(
        const std::string& printer_name,
        const std::string& action,
        int param = 0,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    // ========== 文本设置服务方法 ==========

    /**
     * @brief 通过 printer/set_text 服务设置要打印的文本
     * @param printer_name 打印机名称 (left/center/right/all)
     * @param text 要打印的文本内容
     * @param timeout 超时时间
     * @return (成功标志, 消息) 元组
     */
    std::tuple<bool, std::string> set_text(
        const std::string& printer_name,
        const std::string& text,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    // ========== 便捷方法 - 蜂鸣 ==========

    std::tuple<bool, std::string> beep(
        const std::string& printer_name, 
        int times = 1, 
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    // ========== 便捷方法 - 打印控制 ==========

    std::tuple<bool, std::string> start_print(
        const std::string& printer_name,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    std::tuple<bool, std::string> stop_print(
        const std::string& printer_name,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );


private:
    // 服务客户端
    rclcpp::Client<xline_msgs::srv::PrinterCommand>::SharedPtr send_command_client_;
    rclcpp::Client<xline_msgs::srv::QuickCommand>::SharedPtr quick_command_client_;
    rclcpp::Client<xline_msgs::srv::SetText>::SharedPtr set_text_client_;

};

  } // namespace base_controller
} // namespace xline
