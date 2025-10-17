"""
喷墨打印机 ROS 2 客户端节点

一个完整的ROS 2节点，包含所有打印机服务客户端功能。
"""

import json
from typing import Tuple, Optional, Dict
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from xline_msgs.srv import PrinterCommand, QuickCommand, SetPrinterEnabled


class InkjetClientNode(Node):
    """
    喷墨打印机客户端节点

    继承自Node，实现所有打印机服务的客户端功能。
    支持三个打印机：left、center、right
    """

    def __init__(self):
        super().__init__('inkjet_client_node')

        self.get_logger().info('喷墨打印机客户端节点初始化中...')

        # 创建通用服务客户端
        self._send_command_client = self.create_client(
            PrinterCommand,
            'printer/send_command'
        )

        # 创建快速命令服务客户端
        self._quick_command_client = self.create_client(
            QuickCommand,
            'printer/quick_command'
        )

        # 创建状态查询服务客户端（三个打印机）
        self._status_left_client = self.create_client(
            Trigger,
            'printer_left/status'
        )

        self._status_center_client = self.create_client(
            Trigger,
            'printer_center/status'
        )

        self._status_right_client = self.create_client(
            Trigger,
            'printer_right/status'
        )

        # 创建设置打印机启用状态服务客户端
        self._set_enabled_client = self.create_client(
            SetPrinterEnabled,
            'printer/set_enabled'
        )

        self.get_logger().info('所有服务客户端已创建')
        self.get_logger().info('=' * 60)
        self.get_logger().info('喷墨打印机客户端节点已启动')
        self.get_logger().info('可用服务:')
        self.get_logger().info('  - printer/send_command (通用命令)')
        self.get_logger().info('  - printer/quick_command (快速命令)')
        self.get_logger().info('  - printer/set_enabled (设置打印机启用状态)')
        self.get_logger().info('  - printer_left/status (左打印机状态)')
        self.get_logger().info('  - printer_center/status (中打印机状态)')
        self.get_logger().info('  - printer_right/status (右打印机状态)')
        self.get_logger().info('=' * 60)

    def wait_for_all_services(self, timeout_sec: float = 5.0) -> bool:
        """
        等待所有服务可用

        Args:
            timeout_sec: 超时时间（秒）

        Returns:
            所有服务是否都可用
        """
        self.get_logger().info(f'等待服务可用（超时: {timeout_sec}秒）...')

        services = [
            (self._send_command_client, 'printer/send_command'),
            (self._quick_command_client, 'printer/quick_command'),
            (self._set_enabled_client, 'printer/set_enabled'),
            (self._status_left_client, 'printer_left/status'),
            (self._status_center_client, 'printer_center/status'),
            (self._status_right_client, 'printer_right/status')
        ]

        all_ready = True
        for client, name in services:
            if client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().info(f'  ✓ {name} 可用')
            else:
                self.get_logger().warning(f'  ✗ {name} 不可用')
                all_ready = False

        return all_ready

    # ========== 通用命令服务方法 ==========

    def send_command(
        self,
        printer_name: str,
        command: str,
        json_data: str,
        timeout_sec: float = 3.0
    ) -> Tuple[bool, str]:
        """
        发送通用命令到指定打印机

        Args:
            printer_name: 打印机名称 (left/center/right)
            command: 指令名称或指令码（如 "NOISES", "0x15"）
            json_data: JSON格式的命令数据
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组
        """
        if not self._send_command_client.wait_for_service(timeout_sec=1.0):
            msg = 'printer/send_command 服务不可用'
            self.get_logger().error(msg)
            return False, msg

        request = PrinterCommand.Request()
        request.printer_name = printer_name
        request.command = command
        request.json_data = json_data

        try:
            self.get_logger().info(f'发送命令: printer={printer_name}, cmd={command}')
            future = self._send_command_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'命令成功: {response.message}')
                else:
                    self.get_logger().warning(f'命令失败: {response.message}')
                return response.success, response.message
            else:
                msg = '服务调用超时'
                self.get_logger().error(msg)
                return False, msg

        except Exception as e:
            msg = f'发送命令异常: {str(e)}'
            self.get_logger().error(msg)
            return False, msg

    # ========== 快速命令服务方法 ==========

    def quick_command(
        self,
        printer_name: str,
        action: str,
        param: int = 0,
        timeout_sec: float = 3.0
    ) -> Tuple[bool, str]:
        """
        发送快速命令

        Args:
            printer_name: 打印机名称 (left/center/right/all)
            action: 动作名称 (beep/start_print/stop_print/clean_nozzle/ink_level)
            param: 参数值（如蜂鸣次数、清洗强度）
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组
        """
        if not self._quick_command_client.wait_for_service(timeout_sec=1.0):
            msg = 'printer/quick_command 服务不可用'
            self.get_logger().error(msg)
            return False, msg

        request = QuickCommand.Request()
        request.printer_name = printer_name
        request.action = action
        request.param = param

        try:
            self.get_logger().info(f'快速命令: printer={printer_name}, action={action}, param={param}')
            future = self._quick_command_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'快速命令成功: {response.message}')
                else:
                    self.get_logger().warning(f'快速命令失败: {response.message}')
                return response.success, response.message
            else:
                msg = '服务调用超时'
                self.get_logger().error(msg)
                return False, msg

        except Exception as e:
            msg = f'快速命令异常: {str(e)}'
            self.get_logger().error(msg)
            return False, msg

    # ========== 便捷方法 - 蜂鸣 ==========

    def beep(self, printer_name: str, times: int = 1, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """
        蜂鸣

        Args:
            printer_name: 打印机名称 (left/center/right/all)
            times: 蜂鸣次数
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组
        """
        return self.quick_command(printer_name, 'beep', times, timeout_sec)

    def beep_left(self, times: int = 1, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """左打印机蜂鸣"""
        return self.beep('left', times, timeout_sec)

    def beep_center(self, times: int = 1, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """中打印机蜂鸣"""
        return self.beep('center', times, timeout_sec)

    def beep_right(self, times: int = 1, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """右打印机蜂鸣"""
        return self.beep('right', times, timeout_sec)

    def beep_all(self, times: int = 1, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """所有打印机蜂鸣"""
        return self.beep('all', times, timeout_sec)

    # ========== 便捷方法 - 打印控制 ==========

    def start_print(self, printer_name: str, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """
        开启打印

        Args:
            printer_name: 打印机名称 (left/center/right/all)
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组
        """
        return self.quick_command(printer_name, 'start_print', 0, timeout_sec)

    def start_print_left(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """左打印机开启打印"""
        return self.start_print('left', timeout_sec)

    def start_print_center(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """中打印机开启打印"""
        return self.start_print('center', timeout_sec)

    def start_print_right(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """右打印机开启打印"""
        return self.start_print('right', timeout_sec)

    def start_print_all(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """所有打印机开启打印"""
        return self.start_print('all', timeout_sec)

    def stop_print(self, printer_name: str, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """
        关闭打印

        Args:
            printer_name: 打印机名称 (left/center/right/all)
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组
        """
        return self.quick_command(printer_name, 'stop_print', 0, timeout_sec)

    def stop_print_left(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """左打印机关闭打印"""
        return self.stop_print('left', timeout_sec)

    def stop_print_center(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """中打印机关闭打印"""
        return self.stop_print('center', timeout_sec)

    def stop_print_right(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """右打印机关闭打印"""
        return self.stop_print('right', timeout_sec)

    def stop_print_all(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """所有打印机关闭打印"""
        return self.stop_print('all', timeout_sec)

    # ========== 便捷方法 - 喷头清洗 ==========

    def clean_nozzle(
        self,
        printer_name: str,
        intensity: int = 20,
        timeout_sec: float = 3.0
    ) -> Tuple[bool, str]:
        """
        清洗喷头

        Args:
            printer_name: 打印机名称 (left/center/right/all)
            intensity: 清洗强度
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组
        """
        return self.quick_command(printer_name, 'clean_nozzle', intensity, timeout_sec)

    def clean_nozzle_left(self, intensity: int = 20, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """左打印机清洗喷头"""
        return self.clean_nozzle('left', intensity, timeout_sec)

    def clean_nozzle_center(self, intensity: int = 20, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """中打印机清洗喷头"""
        return self.clean_nozzle('center', intensity, timeout_sec)

    def clean_nozzle_right(self, intensity: int = 20, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """右打印机清洗喷头"""
        return self.clean_nozzle('right', intensity, timeout_sec)

    def clean_nozzle_all(self, intensity: int = 20, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """所有打印机清洗喷头"""
        return self.clean_nozzle('all', intensity, timeout_sec)

    # ========== 便捷方法 - 墨盒查询 ==========

    def query_ink_level(self, printer_name: str, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """
        查询墨盒模量

        Args:
            printer_name: 打印机名称 (left/center/right/all)
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组
        """
        return self.quick_command(printer_name, 'ink_level', 0, timeout_sec)

    def query_ink_level_left(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """查询左打印机墨盒模量"""
        return self.query_ink_level('left', timeout_sec)

    def query_ink_level_center(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """查询中打印机墨盒模量"""
        return self.query_ink_level('center', timeout_sec)

    def query_ink_level_right(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """查询右打印机墨盒模量"""
        return self.query_ink_level('right', timeout_sec)

    def query_ink_level_all(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """查询所有打印机墨盒模量"""
        return self.query_ink_level('all', timeout_sec)

    # ========== 状态查询服务方法 ==========

    def get_status_left(self, timeout_sec: float = 3.0) -> Tuple[bool, Optional[Dict]]:
        """
        获取左打印机状态

        Args:
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 状态字典) 元组
        """
        return self._get_status(self._status_left_client, 'printer_left', timeout_sec)

    def get_status_center(self, timeout_sec: float = 3.0) -> Tuple[bool, Optional[Dict]]:
        """
        获取中打印机状态

        Args:
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 状态字典) 元组
        """
        return self._get_status(self._status_center_client, 'printer_center', timeout_sec)

    def get_status_right(self, timeout_sec: float = 3.0) -> Tuple[bool, Optional[Dict]]:
        """
        获取右打印机状态

        Args:
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 状态字典) 元组
        """
        return self._get_status(self._status_right_client, 'printer_right', timeout_sec)

    def get_status_all(self, timeout_sec: float = 3.0) -> Dict[str, Tuple[bool, Optional[Dict]]]:
        """
        获取所有打印机状态

        Args:
            timeout_sec: 超时时间（秒）

        Returns:
            包含所有打印机状态的字典
        """
        return {
            'left': self.get_status_left(timeout_sec),
            'center': self.get_status_center(timeout_sec),
            'right': self.get_status_right(timeout_sec)
        }

    def _get_status(
        self,
        client,
        printer_name: str,
        timeout_sec: float
    ) -> Tuple[bool, Optional[Dict]]:
        """
        获取打印机状态的通用方法

        Args:
            client: 服务客户端
            printer_name: 打印机名称
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 状态字典) 元组
        """
        if not client.wait_for_service(timeout_sec=1.0):
            msg = f'{printer_name}/status 服务不可用'
            self.get_logger().error(msg)
            return False, None

        request = Trigger.Request()

        try:
            self.get_logger().info(f'查询 {printer_name} 状态')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    try:
                        status_dict = json.loads(response.message)
                        self.get_logger().info(f'{printer_name} 状态: {response.message}')
                        return True, status_dict
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f'状态JSON解析失败: {e}')
                        return False, None
                else:
                    self.get_logger().warning(f'{printer_name} 状态查询失败: {response.message}')
                    return False, None
            else:
                msg = '服务调用超时'
                self.get_logger().error(msg)
                return False, None

        except Exception as e:
            msg = f'状态查询异常: {str(e)}'
            self.get_logger().error(msg)
            return False, None

    # ========== 设置打印机启用状态服务方法 ==========

    def set_printer_enabled(
        self,
        printer_name: str,
        enabled: bool,
        timeout_sec: float = 3.0
    ) -> Tuple[bool, str]:
        """
        设置打印机启用状态

        Args:
            printer_name: 打印机名称 (left/center/right/all)
            enabled: 是否启用
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组

        Examples:
            >>> # 禁用左打印机
            >>> success, msg = node.set_printer_enabled('left', False)
            >>>
            >>> # 启用中打印机
            >>> success, msg = node.set_printer_enabled('center', True)
            >>>
            >>> # 禁用所有打印机
            >>> success, msg = node.set_printer_enabled('all', False)
        """
        if not self._set_enabled_client.wait_for_service(timeout_sec=1.0):
            msg = 'printer/set_enabled 服务不可用'
            self.get_logger().error(msg)
            return False, msg

        request = SetPrinterEnabled.Request()
        request.printer_name = printer_name
        request.enabled = enabled

        try:
            self.get_logger().info(f'设置打印机启用状态: printer={printer_name}, enabled={enabled}')
            future = self._set_enabled_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'设置成功: {response.message}')
                else:
                    self.get_logger().warning(f'设置失败: {response.message}')
                return response.success, response.message
            else:
                msg = '服务调用超时'
                self.get_logger().error(msg)
                return False, msg

        except Exception as e:
            msg = f'设置异常: {str(e)}'
            self.get_logger().error(msg)
            return False, msg

    # 便捷方法 - 启用/禁用打印机

    def enable_printer_left(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """启用左打印机"""
        return self.set_printer_enabled('left', True, timeout_sec)

    def disable_printer_left(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """禁用左打印机"""
        return self.set_printer_enabled('left', False, timeout_sec)

    def enable_printer_center(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """启用中打印机"""
        return self.set_printer_enabled('center', True, timeout_sec)

    def disable_printer_center(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """禁用中打印机"""
        return self.set_printer_enabled('center', False, timeout_sec)

    def enable_printer_right(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """启用右打印机"""
        return self.set_printer_enabled('right', True, timeout_sec)

    def disable_printer_right(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """禁用右打印机"""
        return self.set_printer_enabled('right', False, timeout_sec)

    def enable_all_printers(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """启用所有打印机"""
        return self.set_printer_enabled('all', True, timeout_sec)

    def disable_all_printers(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """禁用所有打印机"""
        return self.set_printer_enabled('all', False, timeout_sec)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    node = InkjetClientNode()

    # 等待所有服务可用
    if node.wait_for_all_services(timeout_sec=5.0):
        node.get_logger().info('所有服务已就绪，客户端节点运行中...')
    else:
        node.get_logger().warning('部分服务不可用，但节点继续运行')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号')
    finally:
        node.get_logger().info('正在关闭节点...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
