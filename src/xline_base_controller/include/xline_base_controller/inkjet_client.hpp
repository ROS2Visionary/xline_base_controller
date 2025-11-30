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
 *     auto [success, msg] = node->test_print_left();
 *     if (success) {
 *         RCLCPP_INFO(node->get_logger(), "左打印机测试流程执行成功");
 *         RCLCPP_INFO(node->get_logger(), "%s", msg.c_str());
 *     } else {
 *         RCLCPP_ERROR(node->get_logger(), "测试失败: %s", msg.c_str());
 *     }
 *     
 *     // 对所有打印机执行测试
 *     auto [success_all, msg_all] = node->test_print_all();
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
#include <xline_msgs/srv/set_printer_enabled.hpp>
#include <xline_msgs/srv/set_printer_active.hpp>

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

    // ========== 便捷方法 - 打印模式设置 ==========

    /**
     * @brief 设置打印模式（通过 QuickCommand:set_print_mode）
     *
     * 使用快速命令服务设置打印模式参数，对应 JSON 中的 PrintMode:
     * - interval  -> PrintMode.interval
     * - is_full_end -> PrintMode.isFullEnd
     * - mode -> PrintMode.mode
     *
     * @param printer_name 打印机名称 (left/center/right/all)
     * @param interval 打印间隔（毫秒），默认75ms
     * @param is_full_end 是否整段结束标志（协议字段 isFullEnd），默认0
     * @param mode 打印模式（协议字段 mode），默认1
     * @param timeout 超时时间
     * @return (成功标志, 消息) 元组
     */
    std::tuple<bool, std::string> set_print_mode(
        const std::string& printer_name,
        int interval = 75,
        int is_full_end = 0,
        int mode = 1,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    // 便捷方法 - 各打印机打印模式设置
    std::tuple<bool, std::string> set_print_mode_left(
        int interval = 75,
        int is_full_end = 0,
        int mode = 1,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    std::tuple<bool, std::string> set_print_mode_center(
        int interval = 75,
        int is_full_end = 0,
        int mode = 1,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    std::tuple<bool, std::string> set_print_mode_right(
        int interval = 75,
        int is_full_end = 0,
        int mode = 1,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    std::tuple<bool, std::string> set_print_mode_all(
        int interval = 75,
        int is_full_end = 0,
        int mode = 1,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    // ========== 便捷方法 - 单条线段测试（通过 QuickCommand:single_line） ==========

    /**
     * @brief 发送单条线段测试消息
     *
     * 通过快速命令服务调用 async_inkjet_node 中的 single_line 流程，
     * 在打印机端发送一条 TEST 指令，内容为单条线段。
     *
     * @param printer_name 打印机名称 (left/center/right/all)
     * @param height 线段高度（像素），默认5
     * @param width  线段宽度（像素），默认150
     * @param x      起始 X 坐标，默认0
     * @param y      起始 Y 坐标，默认75
     * @param timeout 超时时间
     * @return (成功标志, 消息) 元组
     */
    std::tuple<bool, std::string> send_single_line(
        const std::string& printer_name,
        int height = 5,
        int width = 150,
        int x = 0,
        int y = 75,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    // 便捷方法 - 各打印机单条线段测试
    std::tuple<bool, std::string> send_single_line_left(
        int height = 5,
        int width = 150,
        int x = 0,
        int y = 75,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    std::tuple<bool, std::string> send_single_line_center(
        int height = 5,
        int width = 150,
        int x = 0,
        int y = 75,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    std::tuple<bool, std::string> send_single_line_right(
        int height = 5,
        int width = 150,
        int x = 0,
        int y = 75,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    std::tuple<bool, std::string> send_single_line_all(
        int height = 5,
        int width = 150,
        int x = 0,
        int y = 75,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    // ========== 便捷方法 - 蜂鸣 ==========

    std::tuple<bool, std::string> beep(
        const std::string& printer_name, 
        int times = 1, 
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );
    
    std::tuple<bool, std::string> beep_left(int times = 1, std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> beep_center(int times = 1, std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> beep_right(int times = 1, std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> beep_all(int times = 1, std::chrono::seconds timeout = std::chrono::seconds(3));

    // ========== 便捷方法 - 打印控制 ==========

    std::tuple<bool, std::string> start_print(const std::string& printer_name, std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> start_print_left(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> start_print_center(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> start_print_right(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> start_print_all(std::chrono::seconds timeout = std::chrono::seconds(3));

    std::tuple<bool, std::string> stop_print(const std::string& printer_name, std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> stop_print_left(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> stop_print_center(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> stop_print_right(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> stop_print_all(std::chrono::seconds timeout = std::chrono::seconds(3));

    // ========== 便捷方法 - 喷头清洗 ==========

    std::tuple<bool, std::string> clean_nozzle(
        const std::string& printer_name, 
        int intensity = 20, 
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );
    
    std::tuple<bool, std::string> clean_nozzle_left(int intensity = 20, std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> clean_nozzle_center(int intensity = 20, std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> clean_nozzle_right(int intensity = 20, std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> clean_nozzle_all(int intensity = 20, std::chrono::seconds timeout = std::chrono::seconds(3));

    // ========== 便捷方法 - 墨盒查询 ==========

    std::tuple<bool, std::string> query_ink_level(const std::string& printer_name, std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> query_ink_level_left(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> query_ink_level_center(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> query_ink_level_right(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> query_ink_level_all(std::chrono::seconds timeout = std::chrono::seconds(3));

    // ========== 便捷方法 - 测试打印 ==========

    /**
     * @brief 测试打印（完整流程）
     * 
     * 执行完整的测试打印流程，一键触发以下步骤：
     * 1. 设置打印模式（interval=75ms）
     * 2. 等待1秒
     * 3. 发送测试指令（包含文本和装饰图形，13个模块）
     * 4. 等待1秒
     * 5. 开始打印
     * 
     * 这是一个自动化的测试流程，用于全面验证打印机功能。
     * 
     * @param printer_name 打印机名称 (left/center/right/all)
     * @param timeout 超时时间，默认8秒（包含2秒等待时间）
     * @return (成功标志, 消息) 元组
     */
    std::tuple<bool, std::string> test_print(const std::string& printer_name, std::chrono::seconds timeout = std::chrono::seconds(8));
    std::tuple<bool, std::string> test_print_left(std::chrono::seconds timeout = std::chrono::seconds(8));
    std::tuple<bool, std::string> test_print_center(std::chrono::seconds timeout = std::chrono::seconds(8));
    std::tuple<bool, std::string> test_print_right(std::chrono::seconds timeout = std::chrono::seconds(8));
    std::tuple<bool, std::string> test_print_all(std::chrono::seconds timeout = std::chrono::seconds(8));

    // ========== 状态查询服务方法 ==========

    /**
     * @brief 获取打印机状态
     * @param timeout 超时时间
     * @return (成功标志, 状态JSON对象) 元组
     */
    std::tuple<bool, std::optional<json>> get_status_left(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::optional<json>> get_status_center(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::optional<json>> get_status_right(std::chrono::seconds timeout = std::chrono::seconds(3));
    
    /**
     * @brief 获取所有打印机状态
     * @param timeout 超时时间
     * @return 包含所有打印机状态的map
     */
    std::map<std::string, std::tuple<bool, std::optional<json>>> get_status_all(
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    // ========== 设置打印机启用状态服务方法 ==========

    /**
     * @brief 设置打印机自动连接状态
     * 
     * 当设置为 true 时，打印机会自动连接并保持连接。
     * 当设置为 false 时，打印机会断开连接并停止自动重连。
     * 修改会立即生效并持久化到配置文件。
     * 
     * @param printer_name 打印机名称 (left/center/right/all)
     * @param enabled 是否启用自动连接
     * @param timeout 超时时间
     * @return (成功标志, 消息) 元组
     */
    std::tuple<bool, std::string> set_printer_enabled(
        const std::string& printer_name,
        bool enabled,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    // 便捷方法 - 启用/禁用打印机自动连接
    std::tuple<bool, std::string> enable_printer_left(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> disable_printer_left(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> enable_printer_center(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> disable_printer_center(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> enable_printer_right(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> disable_printer_right(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> enable_all_printers(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> disable_all_printers(std::chrono::seconds timeout = std::chrono::seconds(3));

    // ========== 设置打印机激活状态服务方法（功能控制层） ==========

    /**
     * @brief 设置打印机激活状态（功能控制层）
     * 
     * 当设置为 true 时，打印机允许发送指令（前提是已连接）。
     * 当设置为 false 时，打印机禁止发送指令（即使已连接）。
     * 修改会立即生效并持久化到配置文件。
     * 
     * 区别于 set_printer_enabled（auto_connect 控制层）：
     * - set_printer_enabled: 控制是否自动连接（物理连接层）
     * - set_printer_active: 控制是否允许发送指令（逻辑权限层）
     * 
     * @param printer_name 打印机名称 (left/center/right/all)
     * @param active 是否激活（允许发送指令）
     * @param timeout 超时时间
     * @return (成功标志, 消息) 元组
     */
    std::tuple<bool, std::string> set_printer_active(
        const std::string& printer_name,
        bool active,
        std::chrono::seconds timeout = std::chrono::seconds(3)
    );

    // 便捷方法 - 激活/禁用打印机
    std::tuple<bool, std::string> activate_printer_left(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> deactivate_printer_left(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> activate_printer_center(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> deactivate_printer_center(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> activate_printer_right(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> deactivate_printer_right(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> activate_all_printers(std::chrono::seconds timeout = std::chrono::seconds(3));
    std::tuple<bool, std::string> deactivate_all_printers(std::chrono::seconds timeout = std::chrono::seconds(3));

private:
    // 服务客户端
    rclcpp::Client<xline_msgs::srv::PrinterCommand>::SharedPtr send_command_client_;
    rclcpp::Client<xline_msgs::srv::QuickCommand>::SharedPtr quick_command_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr status_left_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr status_center_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr status_right_client_;
    rclcpp::Client<xline_msgs::srv::SetPrinterEnabled>::SharedPtr set_enabled_client_;
    rclcpp::Client<xline_msgs::srv::SetPrinterActive>::SharedPtr set_active_client_;

    /**
     * @brief 获取打印机状态的通用方法
     * @param client 服务客户端
     * @param printer_name 打印机名称
     * @param timeout 超时时间
     * @return (成功标志, 状态JSON对象) 元组
     */
    std::tuple<bool, std::optional<json>> get_status(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
        const std::string& printer_name,
        std::chrono::seconds timeout
    );
};

  } // namespace base_controller
} // namespace xline
