"""
异步喷墨打印机 ROS 2 节点

基于 asyncio 的高性能打印机控制节点。
"""

import asyncio
import threading
from typing import Dict, Optional
import json

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String
from xline_msgs.srv import PrinterCommand

from .async_tcp_client import AsyncTcpClient
from .protocol import InkjetCommand


class AsyncInkjetPrinterNode(Node):
    """
    异步喷墨打印机节点

    特性：
    - 管理多路独立TCP连接（左/中/右打印机）
    - 基于asyncio的高性能I/O
    - 配置热更新
    - ROS 2服务接口
    - 实时状态发布
    """

    def __init__(self) -> None:
        super().__init__('inkjet_printer_node')

        # 声明参数
        self.declare_parameter('config_file', 'printers.yaml')
        self.declare_parameter('status_publish_rate', 1.0)  # Hz
        self.declare_parameter('device_id_left', 0)
        self.declare_parameter('device_id_center', 0)
        self.declare_parameter('device_id_right', 0)

        # 获取参数
        config_file = self.get_parameter('config_file').value
        status_rate = self.get_parameter('status_publish_rate').value
        device_id_left = self.get_parameter('device_id_left').value
        device_id_center = self.get_parameter('device_id_center').value
        device_id_right = self.get_parameter('device_id_right').value

        # TCP 客户端字典
        self._tcp_clients: Dict[str, AsyncTcpClient] = {}

        # 创建三路打印机客户端（每个打印机可以有不同的设备号）
        client_configs = {
            'printer_left': device_id_left,
            'printer_center': device_id_center,
            'printer_right': device_id_right
        }

        for name, device_id in client_configs.items():
            client = AsyncTcpClient(
                logger=self.get_logger(),
                config_name=config_file,
                name=name,
                section=name,
                device_id=device_id
            )

            # 设置回调
            client.set_frame_callback(self._create_frame_callback(name))
            client.set_state_callback(self._create_state_callback(name))

            self._tcp_clients[name] = client

        # ROS 2 发布者
        self._status_pub = self.create_publisher(String, 'printer_status', 10)

        # ROS 2 服务 - 发送命令
        # 通用服务（使用 printer_name 字段路由）
        self._send_generic_srv = self.create_service(
            PrinterCommand, 'printer/send_command', self._handle_send_command_generic
        )

        # 独立服务（向后兼容，忽略 printer_name 字段）
        self._send_left_srv = self.create_service(
            PrinterCommand, 'printer_left/send_command', self._handle_send_left
        )
        self._send_center_srv = self.create_service(
            PrinterCommand, 'printer_center/send_command', self._handle_send_center
        )
        self._send_right_srv = self.create_service(
            PrinterCommand, 'printer_right/send_command', self._handle_send_right
        )

        # ROS 2 服务 - 状态查询
        self._status_left_srv = self.create_service(
            Trigger, 'printer_left/status', self._handle_status_left
        )
        self._status_center_srv = self.create_service(
            Trigger, 'printer_center/status', self._handle_status_center
        )
        self._status_right_srv = self.create_service(
            Trigger, 'printer_right/status', self._handle_status_right
        )

        # ROS 2 定时器 - 发布状态
        self._status_timer = self.create_timer(
            1.0 / status_rate,
            self._publish_status
        )

        # asyncio 事件循环（在独立线程运行）
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._async_thread: Optional[threading.Thread] = None
        self._shutdown_event = threading.Event()

        # 启动 asyncio 事件循环
        self._start_async_loop()

        # 启动所有客户端
        for client in self._tcp_clients.values():
            asyncio.run_coroutine_threadsafe(client.start(), self._loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('异步喷墨打印机节点已启动')
        self.get_logger().info(f'配置文件: {config_file}')
        self.get_logger().info(f'管理的打印机: {", ".join(client_configs.keys())}')
        self.get_logger().info(f'设备号映射: {client_configs}')
        self.get_logger().info('=' * 60)

    def _start_async_loop(self) -> None:
        """在独立线程启动 asyncio 事件循环"""
        def run_loop():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)

            self.get_logger().debug('asyncio 事件循环已启动')

            # 运行直到收到停止信号
            while not self._shutdown_event.is_set():
                try:
                    self._loop.run_until_complete(asyncio.sleep(0.1))
                except Exception as e:
                    self.get_logger().error(f'asyncio 循环异常: {e}')

            # 清理
            try:
                pending = asyncio.all_tasks(self._loop)
                for task in pending:
                    task.cancel()
                self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                self._loop.close()
            except Exception as e:
                self.get_logger().error(f'清理 asyncio 循环异常: {e}')

            self.get_logger().debug('asyncio 事件循环已停止')

        self._async_thread = threading.Thread(target=run_loop, daemon=True)
        self._async_thread.start()

        # 等待循环启动
        import time
        timeout = 5.0
        start_time = time.time()
        while self._loop is None and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if self._loop is None:
            raise RuntimeError('asyncio 事件循环启动超时')

    def _create_frame_callback(self, printer_name: str):
        """创建帧接收回调函数"""
        async def callback(frame: Dict):
            self.get_logger().info(
                f'[{printer_name}] 收到帧: '
                f'设备={frame["device_id"]}, '
                f'指令={frame["command_name"]}, '
                f'JSON={frame.get("json_data", "N/A")}'
            )
        return callback

    def _create_state_callback(self, printer_name: str):
        """创建状态变化回调函数"""
        async def callback(connected: bool):
            state = "已连接" if connected else "已断开"
            self.get_logger().info(f'[{printer_name}] 状态变化: {state}')
        return callback

    def _publish_status(self) -> None:
        """发布状态信息"""
        status_data = {}

        for name, client in self._tcp_clients.items():
            status_data[name] = {
                'connected': client.is_connected(),
                'enabled': client.is_enabled(),
                'status': client.get_status(),
                'device_id': client.get_device_id()
            }

        # 发布为JSON字符串
        msg = String()
        msg.data = json.dumps(status_data, ensure_ascii=False, indent=2)
        self._status_pub.publish(msg)

    # 服务处理函数 - 发送命令
    def _handle_send_command_generic(self, request, response):
        """
        通用命令发送处理（使用 printer_name 字段路由）
        """
        # 解析打印机名称
        printer_name_raw = request.printer_name.strip().lower()

        # 规范化名称：支持 "left"、"printer_left" 等格式
        if printer_name_raw in ['left', 'printer_left']:
            printer_name = 'printer_left'
        elif printer_name_raw in ['center', 'printer_center']:
            printer_name = 'printer_center'
        elif printer_name_raw in ['right', 'printer_right']:
            printer_name = 'printer_right'
        else:
            response.success = False
            response.message = f'未知的打印机标识: {request.printer_name}，支持: left/center/right'
            return response

        return self._handle_send_command(printer_name, request, response)

    def _handle_send_left(self, request, response):
        return self._handle_send_command('printer_left', request, response)

    def _handle_send_center(self, request, response):
        return self._handle_send_command('printer_center', request, response)

    def _handle_send_right(self, request, response):
        return self._handle_send_command('printer_right', request, response)

    def _handle_send_command(self, printer_name: str, request, response):
        """
        通用命令发送处理

        从 request 中解析 command 和 json_data，并发送到打印机
        """
        client = self._tcp_clients.get(printer_name)
        if not client:
            response.success = False
            response.message = f'打印机 {printer_name} 不存在'
            return response

        if not client.is_connected():
            response.success = False
            response.message = f'{printer_name} 未连接'
            return response

        # 解析指令码
        try:
            command_str = request.command.strip()

            # 支持指令名称（如 "NOISES"）或十六进制（如 "0x15"）
            if command_str.upper().startswith('0X'):
                # 十六进制格式：0x15
                command_code = int(command_str, 16)
            elif command_str.isdigit():
                # 十进制数字：21
                command_code = int(command_str)
            else:
                # 指令名称：NOISES
                try:
                    command_code = InkjetCommand[command_str.upper()].value
                except KeyError:
                    response.success = False
                    response.message = f'未知指令: {command_str}'
                    return response

        except (ValueError, AttributeError) as e:
            response.success = False
            response.message = f'指令格式错误: {command_str}, {str(e)}'
            return response

        # 解析 JSON 数据
        try:
            json_data = json.loads(request.json_data)
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'JSON 解析失败: {str(e)}'
            return response

        # 在 asyncio 循环中执行发送命令
        future = asyncio.run_coroutine_threadsafe(
            client.send_command(command_code, json_data),
            self._loop
        )

        try:
            result = future.result(timeout=3.0)
            response.success = result

            if result:
                cmd_name = self._get_command_name(command_code)
                response.message = f'命令已发送: {cmd_name}(0x{command_code:02X}), 数据: {json_data}'
            else:
                response.message = '发送失败'

        except Exception as e:
            response.success = False
            response.message = f'发送异常: {str(e)}'

        return response

    def _get_command_name(self, command_code: int) -> str:
        """获取指令名称"""
        try:
            return InkjetCommand(command_code).name
        except ValueError:
            return f'UNKNOWN'

    # 服务处理函数 - 状态查询
    def _handle_status_left(self, request, response):
        return self._handle_status('printer_left', request, response)

    def _handle_status_center(self, request, response):
        return self._handle_status('printer_center', request, response)

    def _handle_status_right(self, request, response):
        return self._handle_status('printer_right', request, response)

    def _handle_status(self, printer_name: str, request, response):
        """通用状态查询处理"""
        client = self._tcp_clients.get(printer_name)
        if not client:
            response.success = False
            response.message = f'打印机 {printer_name} 不存在'
            return response

        status_info = {
            'name': printer_name,
            'connected': client.is_connected(),
            'enabled': client.is_enabled(),
            'status': client.get_status(),
            'device_id': client.get_device_id()
        }

        response.success = True
        response.message = json.dumps(status_info, ensure_ascii=False, indent=2)
        return response

    def destroy_node(self) -> bool:
        """优雅关闭节点"""
        self.get_logger().info('正在关闭节点...')

        # 停止所有客户端
        for name, client in self._tcp_clients.items():
            self.get_logger().info(f'停止 {name}...')
            future = asyncio.run_coroutine_threadsafe(client.stop(), self._loop)
            try:
                future.result(timeout=5.0)
            except Exception as e:
                self.get_logger().error(f'停止 {name} 失败: {e}')

        # 停止 asyncio 循环
        self._shutdown_event.set()
        if self._async_thread and self._async_thread.is_alive():
            self._async_thread.join(timeout=5.0)

        self.get_logger().info('节点已关闭')
        return super().destroy_node()


def main(args=None) -> None:
    """主函数"""
    rclpy.init(args=args)

    node = AsyncInkjetPrinterNode()

    # 使用多线程执行器
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
