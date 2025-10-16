"""
喷墨打印机 ROS 2 客户端工具类

提供简洁的 Python API，方便在其他 ROS 2 节点中调用打印机服务。
"""

import json
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from xline_msgs.srv import PrinterCommand, QuickCommand
from .command_templates import PrinterCommandTemplates


class PrinterClient:
    """
    打印机客户端工具类

    提供便捷的打印机控制接口，封装了 ROS 2 服务调用细节。

    Examples:
        >>> # 在你的 ROS 2 节点中使用
        >>> client = PrinterClient(your_node, 'printer_left')
        >>>
        >>> # 蜂鸣
        >>> success, msg = client.beep()
        >>>
        >>> # 开启打印
        >>> success, msg = client.start_print()
        >>>
        >>> # 清洗喷头
        >>> success, msg = client.clean_nozzle()
    """

    def __init__(self, node: Node, printer_name: str = 'printer_left'):
        """
        初始化打印机客户端

        Args:
            node: ROS 2 节点实例
            printer_name: 打印机名称 ('printer_left', 'printer_center', 'printer_right', 'left', 'center', 'right')
        """
        self.node = node
        # 规范化打印机名称
        if printer_name in ['left', 'printer_left']:
            self.printer_name = 'left'
        elif printer_name in ['center', 'printer_center']:
            self.printer_name = 'center'
        elif printer_name in ['right', 'printer_right']:
            self.printer_name = 'right'
        else:
            self.printer_name = printer_name

        self._logger = node.get_logger()

        # 创建服务客户端 - 统一便捷服务
        self._quick_command_client = node.create_client(
            QuickCommand, 'printer/quick_command'
        )

        # 创建服务客户端 - 状态查询
        self._status_client = node.create_client(
            Trigger, f'printer_{self.printer_name}/status'
        )

        # 创建服务客户端 - 通用服务
        self._send_command_client = node.create_client(
            PrinterCommand, 'printer/send_command'
        )

    def wait_for_services(self, timeout_sec: float = 5.0) -> bool:
        """
        等待所有服务可用

        Args:
            timeout_sec: 超时时间（秒）

        Returns:
            是否所有服务都可用
        """
        clients = [
            self._quick_command_client,
            self._status_client,
            self._send_command_client
        ]

        all_ready = True
        for client in clients:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self._logger.warning(f'服务 {client.srv_name} 不可用')
                all_ready = False

        return all_ready

    def beep(self, times: int = 1, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """
        蜂鸣

        Args:
            times: 蜂鸣次数
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组

        Examples:
            >>> success, msg = client.beep()
            >>> if success:
            >>>     print("蜂鸣成功")
            >>> success, msg = client.beep(times=3)  # 蜂鸣3次
        """
        return self._call_quick_command('beep', times, timeout_sec)

    def start_print(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """
        开启打印

        Args:
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组
        """
        return self._call_quick_command('start_print', 0, timeout_sec)

    def stop_print(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """
        关闭打印

        Args:
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组
        """
        return self._call_quick_command('stop_print', 0, timeout_sec)

    def clean_nozzle(self, intensity: int = 20, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """
        清洗喷头

        Args:
            intensity: 清洗强度（默认20）
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组

        Examples:
            >>> success, msg = client.clean_nozzle()
            >>> success, msg = client.clean_nozzle(intensity=30)  # 自定义强度
        """
        return self._call_quick_command('clean_nozzle', intensity, timeout_sec)

    def get_status(self, timeout_sec: float = 3.0) -> Tuple[bool, Optional[dict]]:
        """
        获取打印机状态

        Args:
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 状态字典) 元组

        Examples:
            >>> success, status = client.get_status()
            >>> if success:
            >>>     print(f"连接状态: {status['connected']}")
            >>>     print(f"设备号: {status['device_id']}")
        """
        success, msg = self._call_simple_service(self._status_client, '获取状态', timeout_sec)

        if success:
            try:
                status_dict = json.loads(msg)
                return True, status_dict
            except json.JSONDecodeError:
                self._logger.error(f'状态JSON解析失败: {msg}')
                return False, None
        else:
            return False, None

    def send_custom_command(
        self,
        command: str,
        json_data: str,
        timeout_sec: float = 3.0
    ) -> Tuple[bool, str]:
        """
        发送自定义命令（使用通用服务）

        Args:
            command: 指令名称或指令码（如 "NOISES", "0x15"）
            json_data: JSON格式的命令数据
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组

        Examples:
            >>> # 自定义蜂鸣3次
            >>> success, msg = client.send_custom_command(
            >>>     "NOISES",
            >>>     '{"EU2L":{"noises":3}}'
            >>> )
        """
        if not self._send_command_client.wait_for_service(timeout_sec=1.0):
            return False, f'服务 {self._send_command_client.srv_name} 不可用'

        request = PrinterCommand.Request()
        request.printer_name = self.printer_name
        request.command = command
        request.json_data = json_data

        try:
            future = self._send_command_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

            if future.result() is not None:
                response = future.result()
                return response.success, response.message
            else:
                return False, '服务调用超时'

        except Exception as e:
            self._logger.error(f'发送自定义命令异常: {e}')
            return False, f'异常: {str(e)}'

    def _call_quick_command(
        self,
        action: str,
        param: int,
        timeout_sec: float
    ) -> Tuple[bool, str]:
        """
        调用快速命令服务的通用方法

        Args:
            action: 动作名称（beep/start_print/stop_print/clean_nozzle）
            param: 参数值
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组
        """
        if not self._quick_command_client.wait_for_service(timeout_sec=1.0):
            msg = f'快速命令服务不可用'
            self._logger.warning(msg)
            return False, msg

        request = QuickCommand.Request()
        request.printer_name = self.printer_name
        request.action = action
        request.param = param

        try:
            future = self._quick_command_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

            if future.result() is not None:
                response = future.result()
                return response.success, response.message
            else:
                msg = f'[{self.printer_name}] {action} 服务调用超时'
                self._logger.warning(msg)
                return False, msg

        except Exception as e:
            msg = f'[{self.printer_name}] {action} 异常: {str(e)}'
            self._logger.error(msg)
            return False, msg

    def _call_simple_service(
        self,
        client,
        action_name: str,
        timeout_sec: float
    ) -> Tuple[bool, str]:
        """
        调用简单服务（Trigger类型）的通用方法

        Args:
            client: 服务客户端
            action_name: 动作名称（用于日志）
            timeout_sec: 超时时间（秒）

        Returns:
            (成功标志, 消息) 元组
        """
        if not client.wait_for_service(timeout_sec=1.0):
            msg = f'[{self.printer_name}] {action_name}服务不可用'
            self._logger.warning(msg)
            return False, msg

        request = Trigger.Request()

        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

            if future.result() is not None:
                response = future.result()
                return response.success, response.message
            else:
                msg = f'[{self.printer_name}] {action_name}服务调用超时'
                self._logger.warning(msg)
                return False, msg

        except Exception as e:
            msg = f'[{self.printer_name}] {action_name}异常: {str(e)}'
            self._logger.error(msg)
            return False, msg


class MultiPrinterClient:
    """
    多打印机客户端

    同时管理左、中、右三个打印机的客户端。

    Examples:
        >>> multi_client = MultiPrinterClient(your_node)
        >>>
        >>> # 所有打印机蜂鸣
        >>> results = multi_client.beep_all()
        >>>
        >>> # 单个打印机操作
        >>> success, msg = multi_client.left.start_print()
    """

    def __init__(self, node: Node):
        """
        初始化多打印机客户端

        Args:
            node: ROS 2 节点实例
        """
        self.node = node
        self.left = PrinterClient(node, 'printer_left')
        self.center = PrinterClient(node, 'printer_center')
        self.right = PrinterClient(node, 'printer_right')

    def wait_for_services(self, timeout_sec: float = 5.0) -> bool:
        """等待所有打印机服务可用"""
        left_ready = self.left.wait_for_services(timeout_sec)
        center_ready = self.center.wait_for_services(timeout_sec)
        right_ready = self.right.wait_for_services(timeout_sec)

        return left_ready and center_ready and right_ready

    def beep_all(self, timeout_sec: float = 3.0) -> dict:
        """所有打印机蜂鸣"""
        return {
            'left': self.left.beep(timeout_sec),
            'center': self.center.beep(timeout_sec),
            'right': self.right.beep(timeout_sec)
        }

    def start_print_all(self, timeout_sec: float = 3.0) -> dict:
        """所有打印机开启打印"""
        return {
            'left': self.left.start_print(timeout_sec),
            'center': self.center.start_print(timeout_sec),
            'right': self.right.start_print(timeout_sec)
        }

    def stop_print_all(self, timeout_sec: float = 3.0) -> dict:
        """所有打印机关闭打印"""
        return {
            'left': self.left.stop_print(timeout_sec),
            'center': self.center.stop_print(timeout_sec),
            'right': self.right.stop_print(timeout_sec)
        }

    def clean_nozzle_all(self, timeout_sec: float = 3.0) -> dict:
        """所有打印机清洗喷头"""
        return {
            'left': self.left.clean_nozzle(timeout_sec),
            'center': self.center.clean_nozzle(timeout_sec),
            'right': self.right.clean_nozzle(timeout_sec)
        }

    def get_status_all(self, timeout_sec: float = 3.0) -> dict:
        """获取所有打印机状态"""
        return {
            'left': self.left.get_status(timeout_sec),
            'center': self.center.get_status(timeout_sec),
            'right': self.right.get_status(timeout_sec)
        }
