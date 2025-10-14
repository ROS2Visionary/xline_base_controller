#ifndef XLINE_INKJET_PRINTER__INKJET_PRINTER_NODE_HPP_
#define XLINE_INKJET_PRINTER__INKJET_PRINTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <atomic>

namespace xline_inkjet_printer
{

class InkjetPrinterNode : public rclcpp::Node
{
public:
  InkjetPrinterNode();
  ~InkjetPrinterNode();

private:
  /**
   * @brief 使用相对路径加载yaml配置参数
   */
  void updateParameter();

  /**
   * @brief TCP连接循环（在独立线程中运行）
   */
  void tcpConnectionLoop();

  /**
   * @brief 连接到TCP服务器
   * @return 连接成功返回true，否则返回false
   */
  bool connectToServer();

  /**
   * @brief 监控TCP连接状态
   */
  void monitorConnection();

  /**
   * @brief 关闭TCP连接
   */
  void closeTcpConnection();

  /**
   * @brief 定时器回调函数
   */
  void timer_callback();

  // 成员变量
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread tcp_thread_;

  int tcp_socket_;
  std::atomic<bool> is_connected_;
  std::atomic<bool> should_stop_;

  std::string tcp_ip_;
  int tcp_port_;
  int reconnect_interval_;
  int connect_timeout_;
};

}  // namespace xline_inkjet_printer

#endif  // XLINE_INKJET_PRINTER__INKJET_PRINTER_NODE_HPP_
