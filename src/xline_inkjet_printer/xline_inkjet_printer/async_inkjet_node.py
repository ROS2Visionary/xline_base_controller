import asyncio
import threading
from typing import Dict, Optional, Tuple
import json
from pathlib import Path
import yaml
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
from std_msgs.msg import String
from xline_msgs.srv import PrinterCommand, QuickCommand, SetPrinterEnabled, SetPrinterActive

from .async_tcp_client import AsyncTcpClient
from .ink_level_query import InkLevelQuery

# ============================================================================
#  更新导入列表，包含所有需要的 msg_encoder 函数
# ============================================================================
from .msg_encoder import (
    # 控制指令
    encode_beep_json,
    encode_start_print_json,
    encode_stop_print_json,
    # 维护指令
    encode_clean_nozzle_json,
    # 打印模式
    encode_print_mode_json,
    encode_printmode_text_json,
    encode_text_message_json_big,
)


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

    # ========================================================================
    # 打印机名称映射常量
    # ========================================================================
    PRINTER_NAME_MAP = {
        'left': 'printer_left',
        'printer_left': 'printer_left',
        'center': 'printer_center',
        'printer_center': 'printer_center',
        'right': 'printer_right',
        'printer_right': 'printer_right',
    }

    ALL_PRINTERS = ['printer_left', 'printer_center', 'printer_right']

    def __init__(self) -> None:
        super().__init__('inkjet_printer_node')

        # 声明参数
        self.declare_parameter('config_file', 'printers.yaml')
        self.declare_parameter('status_publish_rate', 0.5)  # Hz
        self.declare_parameter('device_id_left', 0)
        self.declare_parameter('device_id_center', 0)
        self.declare_parameter('device_id_right', 0)

        # 声明 auto_connect 参数
        self.declare_parameter('printer_left_auto_connect', True)
        self.declare_parameter('printer_center_auto_connect', True)
        self.declare_parameter('printer_right_auto_connect', True)

        # 声明 enabled 参数
        self.declare_parameter('printer_left_enabled', True)
        self.declare_parameter('printer_center_enabled', True)
        self.declare_parameter('printer_right_enabled', True)

        # 获取参数
        config_file = self.get_parameter('config_file').value
        status_rate = self.get_parameter('status_publish_rate').value
        device_id_left = self.get_parameter('device_id_left').value
        device_id_center = self.get_parameter('device_id_center').value
        device_id_right = self.get_parameter('device_id_right').value

        # 配置文件路径
        self._config_path = Path(__file__).resolve().parent / 'config' / config_file

        # TCP 客户端字典
        self._tcp_clients: Dict[str, AsyncTcpClient] = {}

        # 墨盒查询器字典
        self._ink_queries: Dict[str, InkLevelQuery] = {}

        # 创建三路打印机客户端
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

            client.set_frame_callback(self._create_frame_callback(name))
            client.set_state_callback(self._create_state_callback(name))

            self._tcp_clients[name] = client

            # 创建墨盒查询器
            ip = client._ip
            self._ink_queries[name] = InkLevelQuery(host=ip, port=8010, timeout=3.0)

        # ROS 2 发布者
        self._status_pub = self.create_publisher(String, 'printer_status', 10)

        # ROS 2 服务
        self._send_generic_srv = self.create_service(
            PrinterCommand, 'printer/send_command', self._handle_send_command_generic
        )

        self._quick_command_srv = self.create_service(
            QuickCommand, 'printer/quick_command', self._handle_quick_command
        )

        self._status_left_srv = self.create_service(
            Trigger, 'printer_left/status', self._handle_status_left
        )
        self._status_center_srv = self.create_service(
            Trigger, 'printer_center/status', self._handle_status_center
        )
        self._status_right_srv = self.create_service(
            Trigger, 'printer_right/status', self._handle_status_right
        )

        self._set_enabled_srv = self.create_service(
            SetPrinterEnabled, 'printer/set_enabled', self._handle_set_enabled
        )

        self._set_active_srv = self.create_service(
            SetPrinterActive, 'printer/set_active', self._handle_set_active
        )

        # ROS 2 定时器
        self._status_timer = self.create_timer(
            1.0 / status_rate,
            self._publish_status
        )

        # asyncio 事件循环
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._async_thread: Optional[threading.Thread] = None
        self._shutdown_event = threading.Event()

        # 启动 asyncio 事件循环
        self._start_async_loop()

        # 启动所有客户端
        for client in self._tcp_clients.values():
            asyncio.run_coroutine_threadsafe(client.start(), self._loop)

        # 注册参数回调
        self.add_on_set_parameters_callback(self._parameters_callback)

        self.get_logger().info('=' * 60)
        self.get_logger().info('异步喷墨打印机节点已启动（版）')
        self.get_logger().info(f'配置文件: {config_file}')
        self.get_logger().info(f'管理的打印机: {", ".join(client_configs.keys())}')
        self.get_logger().info('=' * 60)

    # ========================================================================
    # 打印机名称解析辅助方法
    # ========================================================================

    def _normalize_printer_name(self, printer_name_raw: str) -> Tuple[Optional[str], Optional[str]]:
        """
        规范化打印机名称

        Args:
            printer_name_raw: 原始打印机名称（如 'left', 'printer_left'）

        Returns:
            (规范化后的名称, 错误信息) 元组
            成功时返回 (printer_name, None)
            失败时返回 (None, error_message)

        Examples:
            >>> self._normalize_printer_name('left')
            ('printer_left', None)
            >>> self._normalize_printer_name('invalid')
            (None, '未知的打印机: invalid，支持: left/center/right')
        """
        printer_name_raw = printer_name_raw.strip().lower()

        if printer_name_raw in self.PRINTER_NAME_MAP:
            return self.PRINTER_NAME_MAP[printer_name_raw], None
        else:
            return None, f'未知的打印机: {printer_name_raw}，支持: left/center/right'

    def _is_all_printers(self, printer_name_raw: str) -> bool:
        """检查是否是 'all' 关键字"""
        return printer_name_raw.strip().lower() == 'all'

    # ========================================================================
    # 使用 msg_encoder 的内部便捷方法
    # ========================================================================

    async def set_print_mode_text(
        self,
        printer_name: str
    ) -> bool:

        client = self._tcp_clients.get(printer_name)
        if not client:
            self.get_logger().error(f'打印机 {printer_name} 不存在')
            return False

        if not client.is_connected():
            self.get_logger().warning(f'{printer_name} 未连接')
            return False

        if not client.is_enabled():
            self.get_logger().warning(f'{printer_name} 已禁用')
            return False

        # 使用 msg_encoder 构造 JSON
        json_data = encode_printmode_text_json(interval=400,couCount=3)

        try:
            result = await client.send_command(json_data)
            if result:
                self.get_logger().info(
                    f'[{printer_name}] 设置文字打印模式成功'
                )
            else:
                self.get_logger().warning(f'[{printer_name}] 设置文字打印模式失败')
            return result
        except Exception as e:
            self.get_logger().error(f'[{printer_name}] 设置文字打印模式异常: {str(e)}')
            return False

    async def set_print_mode_internal(
        self,
        printer_name: str,
        interval: int = 75,
        is_full_end: int = 0,
        mode: int = 1,
    ) -> bool:
        """
        内部便捷方法：设置打印模式

        使用 msg_encoder.build_print_mode_json() 构造指令。

        Args:
            printer_name: 打印机名称
            interval: 打印间隔（毫秒）
            is_full_end: 是否整段结束标志
            mode: 打印模式

        Returns:
            成功标志
        """
        client = self._tcp_clients.get(printer_name)
        if not client:
            self.get_logger().error(f'打印机 {printer_name} 不存在')
            return False

        if not client.is_connected():
            self.get_logger().warning(f'{printer_name} 未连接')
            return False

        if not client.is_enabled():
            self.get_logger().warning(f'{printer_name} 已禁用')
            return False

        # 使用 msg_encoder 构造 JSON
        json_data = encode_print_mode_json(interval, is_full_end, mode)

        try:
            result = await client.send_command(json_data)
            if result:
                self.get_logger().info(
                    f'[{printer_name}] 设置打印模式成功: interval={interval}ms, '
                    f'isFullEnd={is_full_end}, mode={mode}'
                )
            else:
                self.get_logger().warning(f'[{printer_name}] 设置打印模式失败')
            return result
        except Exception as e:
            self.get_logger().error(f'[{printer_name}] 设置打印模式异常: {str(e)}')
            return False

    async def start_print_internal(self, printer_name: str) -> bool:
        """
        内部便捷方法：开始打印

        使用 msg_encoder.encode_start_print_json() 构造指令。

        Args:
            printer_name: 打印机名称

        Returns:
            成功标志
        """
        client = self._tcp_clients.get(printer_name)
        if not client:
            self.get_logger().error(f'打印机 {printer_name} 不存在')
            return False

        if not client.is_connected():
            self.get_logger().warning(f'{printer_name} 未连接')
            return False

        if not client.is_enabled():
            self.get_logger().warning(f'{printer_name} 已禁用')
            return False

        # 使用 msg_encoder 构造 JSON
        json_data = encode_start_print_json()

        try:
            result = await client.send_command(json_data)
            if result:
                self.get_logger().info(f'[{printer_name}] 开始打印成功')
            else:
                self.get_logger().warning(f'[{printer_name}] 开始打印失败')
            return result
        except Exception as e:
            self.get_logger().error(f'[{printer_name}] 开始打印异常: {str(e)}')
            return False

    async def stop_print_internal(self, printer_name: str) -> bool:
        """
        内部便捷方法：停止打印

        使用 msg_encoder.encode_stop_print_json() 构造指令。

        Args:
            printer_name: 打印机名称

        Returns:
            成功标志
        """
        client = self._tcp_clients.get(printer_name)
        if not client:
            self.get_logger().error(f'打印机 {printer_name} 不存在')
            return False

        if not client.is_connected():
            self.get_logger().warning(f'{printer_name} 未连接')
            return False

        if not client.is_enabled():
            self.get_logger().warning(f'{printer_name} 已禁用')
            return False

        # 使用 msg_encoder 构造 JSON
        json_data = encode_stop_print_json()

        try:
            result = await client.send_command(json_data)
            if result:
                self.get_logger().info(f'[{printer_name}] 停止打印成功')
            else:
                self.get_logger().warning(f'[{printer_name}] 停止打印失败')
            return result
        except Exception as e:
            self.get_logger().error(f'[{printer_name}] 停止打印异常: {str(e)}')
            return False

    async def execute_test_print_sequence(self, printer_name: str) -> bool:
        """
        执行完整的测试打印流程

        使用 msg_encoder.build_test_print_json() 构造测试指令。

        流程：
        1. 设置打印模式
        2. 等待2秒
        3. 发送测试指令
        4. 等待2秒
        5. 开始打印

        Args:
            printer_name: 打印机名称

        Returns:
            成功标志
        """
        self.get_logger().info(f'[{printer_name}] 开始执行测试打印流程...')

        client = self._tcp_clients.get(printer_name)
        if not client:
            self.get_logger().error(f'[{printer_name}] 打印机不存在')
            return False

        if not client.is_connected():
            self.get_logger().error(f'[{printer_name}] 未连接')
            return False

        if not client.is_enabled():
            self.get_logger().error(f'[{printer_name}] 已禁用')
            return False

        try:
            # 特殊处理：中间墨盒只需开始打印，不执行完整测试流程
            if printer_name == 'printer_center':
                self.get_logger().info(f'[{printer_name}] 中间墨盒，直接开始打印（跳过测试流程）')
                if not await self.start_print_internal(printer_name):
                    self.get_logger().error(f'[{printer_name}] 开始打印失败')
                    return False
                self.get_logger().info(f'[{printer_name}] ✓ 已直接开始打印')
                return True

            # 步骤1: 设置打印模式
            self.get_logger().info(f'[{printer_name}] 步骤1: 设置文字打印模式')
            if not await self.set_print_mode_text(printer_name):
                self.get_logger().error(f'[{printer_name}] 设置文字打印模式失败')
                return False

            # 步骤2: 等待
            self.get_logger().info(f'[{printer_name}] 步骤2: 等待2秒')
            await asyncio.sleep(2.0)

            # 步骤3: 发送测试指令 - 使用 msg_encoder 构造 JSON
            self.get_logger().info(f'[{printer_name}] 步骤3: 发送测试指令')
            json_data = encode_text_message_json_big("华南农业大学机器人实验室")

            if not await client.send_command(json_data):
                self.get_logger().error(f'[{printer_name}] 发送测试指令失败')
                return False

            # 步骤4: 等待
            self.get_logger().info(f'[{printer_name}] 步骤4: 等待2秒')
            await asyncio.sleep(2.0)

            # 步骤5: 开始打印
            self.get_logger().info(f'[{printer_name}] 步骤5: 开始打印')
            if not await self.start_print_internal(printer_name):
                self.get_logger().error(f'[{printer_name}] 开始打印失败')
                return False

            self.get_logger().info(f'[{printer_name}] ✓ 测试打印流程执行成功')
            return True

        except Exception as e:
            self.get_logger().error(f'[{printer_name}] 测试打印流程异常: {str(e)}')
            return False

    async def send_single_line_message(
        self,
        printer_name: str,
        height: int = 5,
        width: int = 150,
        x: int = 0,
        y: int = 75,
    ) -> bool:
        """
        发送单条线段测试消息

        使用 msg_encoder.build_single_line_json() 构造指令。

        Args:
            printer_name: 打印机名称
            height: 线段高度
            width: 线段宽度
            x: X 坐标
            y: Y 坐标

        Returns:
            成功标志
        """
        self.get_logger().info(
            f'[{printer_name}] 准备发送单条线段测试消息 '
            f'(height={height}, width={width}, x={x}, y={y})...'
        )

        client = self._tcp_clients.get(printer_name)
        if not client:
            self.get_logger().error(f'[{printer_name}] 打印机不存在')
            return False

        if not client.is_connected():
            self.get_logger().error(f'[{printer_name}] 未连接')
            return False

        if not client.is_enabled():
            self.get_logger().error(f'[{printer_name}] 已禁用')
            return False

        # 使用 msg_encoder 构造 JSON
        json_data = build_single_line_json(height, width, x, y)

        try:
            self.get_logger().info(f'[{printer_name}] 发送单条线段测试消息...')
            result = await client.send_command(json_data)
            if result:
                self.get_logger().info(f'[{printer_name}] 单条线段测试消息发送成功')
            else:
                self.get_logger().warning(f'[{printer_name}] 单条线段测试消息发送失败')
            return result
        except Exception as e:
            self.get_logger().error(f'[{printer_name}] 单条线段测试消息发送异常: {str(e)}')
            return False

    # ========================================================================
    # 使用辅助方法简化服务处理函数
    # ========================================================================

    def _handle_send_command_generic(self, request, response):
        """通用命令发送处理（使用辅助方法解析打印机名称）"""
        printer_name, error = self._normalize_printer_name(request.printer_name)

        if error:
            response.success = False
            response.message = error
            self._service_delay(1)
            return response

        return self._handle_send_command(printer_name, request, response)

    def _handle_quick_command(self, request, response):
        """
        处理快速命令请求

        支持的动作: beep, start_print, stop_print, clean_nozzle, test_print, ink_level, set_print_mode
        支持的打印机: left, center, right, all
        """
        action = request.action.lower().strip()
        printer_name_raw = request.printer_name.lower().strip()
        param = request.param if request.param > 0 else None

        # 特殊动作处理
        if action in ['ink_level', 'query_ink', 'ink']:
            return self._handle_ink_level_query(printer_name_raw, response)

        if action in ['test_print', 'test']:
            return self._handle_test_print_sequence(printer_name_raw, response)

        if action in ['single_line', 'line', 'line_test']:
            return self._handle_single_line_quick(printer_name_raw, request.single_line, response)

        if action in ['set_print_mode', 'print_mode', 'mode']:
            interval = request.print_mode.interval if request.print_mode.interval > 0 else (
                param if param is not None else 75
            )
            is_full_end = request.print_mode.is_full_end
            mode = request.print_mode.mode
            return self._handle_set_print_mode_quick(printer_name_raw, interval, is_full_end, mode, response)

        # 标准动作映射
        action_map = {
            'beep': ('蜂鸣', lambda p: encode_beep_json(p if p else 1)),
            'start_print': ('开启打印', lambda p: encode_start_print_json()),
            'start': ('开启打印', lambda p: encode_start_print_json()),
            'stop_print': ('关闭打印', lambda p: encode_stop_print_json()),
            'stop': ('关闭打印', lambda p: encode_stop_print_json()),
            'clean_nozzle': ('清洗喷头', lambda p: encode_clean_nozzle_json(p if p else 20)),
            'clean': ('清洗喷头', lambda p: encode_clean_nozzle_json(p if p else 20)),
        }

        if action not in action_map:
            response.success = False
            response.message = f'不支持的动作: {action}'
            self._service_delay(1)
            return response

        action_name, template_func = action_map[action]

        # 处理 'all' 或单个打印机
        if self._is_all_printers(printer_name_raw):
            results = []
            for pname in self.ALL_PRINTERS:
                try:
                    json_data = template_func(param)
                    temp_response = QuickCommand.Response()
                    self._execute_template_command(pname, json_data, action_name, temp_response)
                    results.append(f'{pname}: {temp_response.message}')
                except Exception as e:
                    results.append(f'{pname}: 错误 - {str(e)}')

            response.success = True
            response.message = '\n'.join(results)
            self._service_delay(1)
            return response
        else:
            printer_name, error = self._normalize_printer_name(printer_name_raw)
            if error:
                response.success = False
                response.message = error
                self._service_delay(1)
                return response

            try:
                json_data = template_func(param)
                return self._execute_template_command(printer_name, json_data, action_name, response)
            except Exception as e:
                response.success = False
                response.message = f'执行失败: {str(e)}'
                self._service_delay(1)
                return response

    # ========================================================================
    # 其他方法保持不变（省略以简化文件长度）
    # ========================================================================

    def _service_delay(self, duration: float) -> None:
        """服务响应延时"""
        time.sleep(duration)

    def _start_async_loop(self) -> None:
        """在独立线程启动 asyncio 事件循环"""
        def run_loop():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)

            while not self._shutdown_event.is_set():
                try:
                    self._loop.run_until_complete(asyncio.sleep(0.1))
                except Exception as e:
                    self.get_logger().error(f'asyncio 循环异常: {e}')

            try:
                pending = asyncio.all_tasks(self._loop)
                for task in pending:
                    task.cancel()
                self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                self._loop.close()
            except Exception as e:
                self.get_logger().error(f'清理 asyncio 循环异常: {e}')

        self._async_thread = threading.Thread(target=run_loop, daemon=True)
        self._async_thread.start()

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
                f'[{printer_name}] 收到帧: 设备={frame["device_id"]}, '
                f'指令={frame["command_name"]}, JSON={frame.get("json_data", "N/A")}'
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
                'auto_connect': client.is_auto_connect(),
                'enabled': client.is_enabled(),
                'is_online': client.is_online(),
                'status': client.get_status(),
                'device_id': client.get_device_id(),
                'ink_level': self._get_ink_level_for_status(name)
            }

        msg = String()
        msg.data = json.dumps(status_data, ensure_ascii=False, indent=2)
        self._status_pub.publish(msg)

    def _get_ink_level_for_status(self, printer_name: str) -> int:
        """获取喷码机墨量余量"""
        return 0  # TODO: 实现真实查询

    def _handle_send_command(self, printer_name: str, request, response):
        """通用命令发送处理"""
        client = self._tcp_clients.get(printer_name)
        if not client:
            response.success = False
            response.message = f'打印机 {printer_name} 不存在'
            self._service_delay(1)
            return response

        if not client.is_connected():
            response.success = False
            response.message = f'{printer_name} 未连接'
            self._service_delay(1)
            return response

        if not client.is_enabled():
            response.success = False
            response.message = f'{printer_name} 已禁用'
            self._service_delay(1)
            return response

        try:
            json_data = json.loads(request.json_data)
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'JSON 解析失败: {str(e)}'
            self._service_delay(1)
            return response

        future = asyncio.run_coroutine_threadsafe(
            client.send_command(json_data),
            self._loop
        )

        try:
            result = future.result(timeout=3.0)
            response.success = result
            response.message = f'命令已发送' if result else '发送失败'
        except Exception as e:
            response.success = False
            response.message = f'发送异常: {str(e)}'

        self._service_delay(1)
        return response

    def _execute_template_command(self, printer_name: str, json_data: dict, action_name: str, response):
        """执行模板命令的通用处理函数"""
        client = self._tcp_clients.get(printer_name)
        if not client:
            response.success = False
            response.message = f'打印机 {printer_name} 不存在'
            self._service_delay(1)
            return response

        if not client.is_connected():
            response.success = False
            response.message = f'{printer_name} 未连接'
            self._service_delay(1)
            return response

        if not client.is_enabled():
            response.success = False
            response.message = f'{printer_name} 已禁用'
            self._service_delay(1)
            return response

        future = asyncio.run_coroutine_threadsafe(
            client.send_command(json_data),
            self._loop
        )

        try:
            result = future.result(timeout=3.0)
            response.success = result
            if result:
                response.message = f'[{printer_name}] {action_name}命令已发送'
            else:
                response.message = f'[{printer_name}] {action_name}命令发送失败'
        except Exception as e:
            response.success = False
            response.message = f'[{printer_name}] {action_name}命令异常: {str(e)}'

        self._service_delay(1)
        return response

    # 状态查询处理函数
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
            self._service_delay(1)
            return response

        ink_level = self._get_ink_level_for_status(printer_name)

        status_info = {
            'name': printer_name,
            'connected': client.is_connected(),
            'auto_connect': client.is_auto_connect(),
            'enabled': client.is_enabled(),
            'is_online': client.is_online(),
            'status': client.get_status(),
            'device_id': client.get_device_id(),
            'ink_level': ink_level
        }

        response.success = True
        response.message = json.dumps(status_info, ensure_ascii=False, indent=2)
        self._service_delay(1)
        return response

    # 其他处理函数使用辅助方法简化...
    # （为简洁起见，省略其他方法的完整实现，但模式相同）

    def _handle_test_print_sequence(self, printer_name_raw: str, response):
        """处理测试打印流程"""
        if self._is_all_printers(printer_name_raw):
            results = []
            for pname in self.ALL_PRINTERS:
                future = asyncio.run_coroutine_threadsafe(
                    self.execute_test_print_sequence(pname),
                    self._loop
                )
                try:
                    success = future.result(timeout=6.0)
                    if success:
                        results.append(f'{pname}: 测试打印流程执行成功 ✓')
                    else:
                        results.append(f'{pname}: 测试打印流程执行失败')
                except Exception as e:
                    results.append(f'{pname}: 异常 - {str(e)}')

            response.success = True
            response.message = '\n'.join(results)
            self._service_delay(1)
            return response
        else:
            printer_name, error = self._normalize_printer_name(printer_name_raw)
            if error:
                response.success = False
                response.message = error
                self._service_delay(1)
                return response

            future = asyncio.run_coroutine_threadsafe(
                self.execute_test_print_sequence(printer_name),
                self._loop
            )

            try:
                success = future.result(timeout=6.0)
                response.success = success
                if success:
                    response.message = f'[{printer_name}] 测试打印流程执行成功 ✓'
                else:
                    response.message = f'[{printer_name}] 测试打印流程执行失败'
            except Exception as e:
                response.success = False
                response.message = f'[{printer_name}] 测试打印流程异常: {str(e)}'

            self._service_delay(1)
            return response

    def _handle_ink_level_query(self, printer_name_raw: str, response):
        """处理墨盒模量查询"""
        if self._is_all_printers(printer_name_raw):
            results = []
            for pname in self.ALL_PRINTERS:
                query = self._ink_queries.get(pname)
                if not query:
                    results.append(f'{pname}: 查询器未初始化')
                    continue

                future = asyncio.run_coroutine_threadsafe(
                    query.query_ink_level(),
                    self._loop
                )

                try:
                    ink_level = future.result(timeout=3.0)
                    if ink_level is not None:
                        results.append(f'{pname}: {ink_level} (0x{ink_level:02X})')
                    else:
                        results.append(f'{pname}: 查询失败')
                except Exception as e:
                    results.append(f'{pname}: 异常 - {str(e)}')

            response.success = True
            response.message = '\n'.join(results)
            self._service_delay(1)
            return response
        else:
            printer_name, error = self._normalize_printer_name(printer_name_raw)
            if error:
                response.success = False
                response.message = error
                self._service_delay(1)
                return response

            query = self._ink_queries.get(printer_name)
            if not query:
                response.success = False
                response.message = f'{printer_name} 查询器未初始化'
                self._service_delay(1)
                return response

            future = asyncio.run_coroutine_threadsafe(
                query.query_ink_level(),
                self._loop
            )

            try:
                ink_level = future.result(timeout=3.0)
                if ink_level is not None:
                    response.success = True
                    response.message = f'[{printer_name}] 墨盒模量: {ink_level} (0x{ink_level:02X})'
                else:
                    response.success = False
                    response.message = f'[{printer_name}] 墨盒模量查询失败'
            except Exception as e:
                response.success = False
                response.message = f'[{printer_name}] 墨盒模量查询异常: {str(e)}'

            self._service_delay(1)
            return response

    def _handle_set_print_mode_quick(self, printer_name_raw: str, interval: int, is_full_end: int, mode: int, response):
        """处理打印模式设置"""
        # 归一化参数
        if interval is None or interval <= 0:
            interval = 75
        if is_full_end is None or is_full_end < 0:
            is_full_end = 0
        if mode is None or mode <= 0:
            mode = 1

        if self._is_all_printers(printer_name_raw):
            results = []
            for printer_name in self.ALL_PRINTERS:
                try:
                    success = self.set_print_mode(printer_name, interval, is_full_end, mode)
                    msg = (f'[{printer_name}] 设置打印模式成功: interval={interval}ms'
                           if success else f'[{printer_name}] 设置打印模式失败')
                    results.append(msg)
                except Exception as e:
                    results.append(f'[{printer_name}] 设置打印模式异常: {str(e)}')

            response.success = True
            response.message = '\n'.join(results)
            self._service_delay(1)
            return response

        printer_name, error = self._normalize_printer_name(printer_name_raw)
        if error:
            response.success = False
            response.message = error
            self._service_delay(1)
            return response

        try:
            success = self.set_print_mode(printer_name, interval, is_full_end, mode)
            response.success = success
            if success:
                response.message = f'[{printer_name}] 设置打印模式成功: interval={interval}ms'
            else:
                response.message = f'[{printer_name}] 设置打印模式失败'
        except Exception as e:
            response.success = False
            response.message = f'[{printer_name}] 设置打印模式异常: {str(e)}'

        self._service_delay(1)
        return response

    def _handle_single_line_quick(self, printer_name_raw: str, cfg, response):
        """处理单条线段测试"""
        height = cfg.height if cfg.height > 0 else 5
        width = cfg.width if cfg.width > 0 else 150
        x = cfg.x if cfg.x != 0 else 0
        y = cfg.y if cfg.y != 0 else 75

        if self._is_all_printers(printer_name_raw):
            results = []
            for pname in self.ALL_PRINTERS:
                future = asyncio.run_coroutine_threadsafe(
                    self.send_single_line_message(pname, height=height, width=width, x=x, y=y),
                    self._loop
                )
                try:
                    success = future.result(timeout=3.0)
                    if success:
                        results.append(f'{pname}: 单条线段测试消息发送成功')
                    else:
                        results.append(f'{pname}: 单条线段测试消息发送失败')
                except Exception as e:
                    results.append(f'{pname}: 异常 - {str(e)}')

            response.success = True
            response.message = '\n'.join(results)
            self._service_delay(1)
            return response

        printer_name, error = self._normalize_printer_name(printer_name_raw)
        if error:
            response.success = False
            response.message = error
            self._service_delay(1)
            return response

        future = asyncio.run_coroutine_threadsafe(
            self.send_single_line_message(printer_name, height=height, width=width, x=x, y=y),
            self._loop
        )

        try:
            success = future.result(timeout=3.0)
            response.success = success
            if success:
                response.message = f'[{printer_name}] 单条线段测试消息发送成功'
            else:
                response.message = f'[{printer_name}] 单条线段测试消息发送失败'
        except Exception as e:
            response.success = False
            response.message = f'[{printer_name}] 异常: {str(e)}'

        self._service_delay(1)
        return response

    # 同步包装方法
    def set_print_mode(self, printer_name: str, interval: int = 75, is_full_end: int = 0, mode: int = 1) -> bool:
        """同步版本：设置打印模式"""
        if self._loop is None:
            return False
        future = asyncio.run_coroutine_threadsafe(
            self.set_print_mode_internal(printer_name, interval, is_full_end, mode),
            self._loop
        )
        try:
            return future.result(timeout=3.0)
        except Exception:
            return False

    def start_print(self, printer_name: str) -> bool:
        """同步版本：开始打印"""
        if self._loop is None:
            return False
        future = asyncio.run_coroutine_threadsafe(
            self.start_print_internal(printer_name),
            self._loop
        )
        try:
            return future.result(timeout=3.0)
        except Exception:
            return False

    def stop_print(self, printer_name: str) -> bool:
        """同步版本：停止打印"""
        if self._loop is None:
            return False
        future = asyncio.run_coroutine_threadsafe(
            self.stop_print_internal(printer_name),
            self._loop
        )
        try:
            return future.result(timeout=3.0)
        except Exception:
            return False

    def _parameters_callback(self, params):
        """
        参数变化回调函数

        当参数通过 ros2 param set 或服务修改时触发
        """
        successful = True
        for param in params:
            if param.name in ['printer_left_auto_connect', 'printer_center_auto_connect', 'printer_right_auto_connect']:
                # 解析打印机名称
                printer_name = param.name.replace('_auto_connect', '')
                auto_connect = param.value

                self.get_logger().info(
                    f'参数变化: {param.name} = {auto_connect}'
                )

                # 更新客户端状态
                client = self._tcp_clients.get(printer_name)
                if client:
                    # 在 asyncio 循环中执行
                    future = asyncio.run_coroutine_threadsafe(
                        client.set_auto_connect(auto_connect),
                        self._loop
                    )
                    try:
                        result = future.result(timeout=3.0)
                        if not result:
                            self.get_logger().error(f'设置 {printer_name} auto_connect 失败')
                            successful = False
                    except Exception as e:
                        self.get_logger().error(f'设置 {printer_name} auto_connect 异常: {e}')
                        successful = False
                else:
                    self.get_logger().error(f'未找到打印机客户端: {printer_name}')
                    successful = False

            elif param.name in ['printer_left_enabled', 'printer_center_enabled', 'printer_right_enabled']:
                # 解析打印机名称
                printer_name = param.name.replace('_enabled', '')
                enabled = param.value

                self.get_logger().info(
                    f'参数变化: {param.name} = {enabled}'
                )

                # 更新客户端状态
                client = self._tcp_clients.get(printer_name)
                if client:
                    # 在 asyncio 循环中执行
                    future = asyncio.run_coroutine_threadsafe(
                        client.set_enabled(enabled),
                        self._loop
                    )
                    try:
                        result = future.result(timeout=3.0)
                        if not result:
                            self.get_logger().error(f'设置 {printer_name} enabled 失败')
                            successful = False
                    except Exception as e:
                        self.get_logger().error(f'设置 {printer_name} enabled 异常: {e}')
                        successful = False
                else:
                    self.get_logger().error(f'未找到打印机客户端: {printer_name}')
                    successful = False

        return SetParametersResult(successful=successful)

    # ========== 服务处理函数 - 设置打印机自动连接状态 ==========
    def _handle_set_enabled(self, request, response):
        """
        处理设置打印机自动连接状态的服务请求

        流程：
        1. 更新 ROS 2 参数（触发参数回调 -> 更新内存 -> 控制连接）
        2. 持久化到 yaml 文件
        """
        printer_name_raw = request.printer_name.lower().strip()
        enabled = request.enabled

        self.get_logger().info(
            f'收到设置自动连接状态请求: printer={printer_name_raw}, auto_connect={enabled}'
        )

        # 解析打印机名称
        if printer_name_raw == 'all':
            # 设置所有打印机
            results = []
            for pname in ['left', 'center', 'right']:
                success, msg = self._set_single_printer_enabled(pname, enabled)
                results.append(f'{pname}: {msg}')

            response.success = True
            response.message = '\n'.join(results)
            self._service_delay(1)
            return response
        else:
            # 单个打印机
            # 规范化名称
            if printer_name_raw in ['left', 'printer_left']:
                printer_short = 'left'
            elif printer_name_raw in ['center', 'printer_center']:
                printer_short = 'center'
            elif printer_name_raw in ['right', 'printer_right']:
                printer_short = 'right'
            else:
                response.success = False
                response.message = f'未知的打印机: {printer_name_raw}，支持: left/center/right/all'
                self._service_delay(1)
                return response

            success, msg = self._set_single_printer_enabled(printer_short, enabled)
            response.success = success
            response.message = msg
            self._service_delay(1)
            return response

    def _set_single_printer_enabled(self, printer_short: str, enabled: bool):
        """
        设置单个打印机的自动连接状态

        Args:
            printer_short: 打印机短名称 (left/center/right)
            enabled: 是否启用自动连接

        Returns:
            (成功标志, 消息) 元组
        """
        printer_name = f'printer_{printer_short}'
        param_name = f'{printer_name}_auto_connect'

        # 1. 更新 ROS 2 参数（会触发参数回调）
        try:
            self.set_parameters([Parameter(param_name, Parameter.Type.BOOL, enabled)])
            self.get_logger().info(f'已更新参数: {param_name} = {enabled}')
        except Exception as e:
            msg = f'更新参数失败: {str(e)}'
            self.get_logger().error(msg)
            return False, msg

        # 2. 持久化到 yaml 文件
        try:
            self._persist_auto_connect_to_yaml(printer_name, enabled)
            msg = f'[{printer_name}] 自动连接状态已设置为 {enabled} 并持久化'
            self.get_logger().info(msg)
            return True, msg
        except Exception as e:
            msg = f'[{printer_name}] 自动连接状态已设置为 {enabled}，但持久化失败: {str(e)}'
            self.get_logger().warning(msg)
            return True, msg  # 即使持久化失败，参数已更新，仍返回成功

    def _persist_auto_connect_to_yaml(self, printer_name: str, auto_connect: bool) -> None:
        """
        持久化 auto_connect 参数到 yaml 文件

        Args:
            printer_name: 打印机名称 (printer_left/printer_center/printer_right)
            auto_connect: 是否启用自动连接
        """
        try:
            # 读取现有配置
            with self._config_path.open('r', encoding='utf-8') as f:
                config = yaml.safe_load(f) or {}

            # 更新 auto_connect 字段
            if 'connections' in config and printer_name in config['connections']:
                config['connections'][printer_name]['auto_connect'] = auto_connect
                self.get_logger().debug(
                    f'更新配置: connections.{printer_name}.auto_connect = {auto_connect}'
                )
            else:
                raise ValueError(f'配置文件中未找到 {printer_name}')

            # 写回文件
            with self._config_path.open('w', encoding='utf-8') as f:
                yaml.dump(
                    config,
                    f,
                    default_flow_style=False,
                    allow_unicode=True,
                    sort_keys=False
                )

            self.get_logger().info(f'已持久化 {printer_name}.auto_connect = {auto_connect} 到 {self._config_path}')

        except Exception as e:
            self.get_logger().error(f'持久化到 yaml 失败: {e}')
            raise

    # ========== 服务处理函数 - 设置打印机激活状态（功能控制层） ==========
    def _handle_set_active(self, request, response):
        """
        处理设置打印机激活状态的服务请求

        流程：
        1. 更新 ROS 2 参数（触发参数回调 -> 更新内存 -> 控制功能）
        2. 持久化到 yaml 文件
        """
        printer_name_raw = request.printer_name.lower().strip()
        active = request.active

        self.get_logger().info(
            f'收到设置激活状态请求: printer={printer_name_raw}, active={active}'
        )

        # 解析打印机名称
        if printer_name_raw == 'all':
            # 设置所有打印机
            results = []
            for pname in ['left', 'center', 'right']:
                success, msg = self._set_single_printer_active(pname, active)
                results.append(f'{pname}: {msg}')

            response.success = True
            response.message = '\n'.join(results)
            self._service_delay(1)
            return response
        else:
            # 单个打印机
            # 规范化名称
            if printer_name_raw in ['left', 'printer_left']:
                printer_short = 'left'
            elif printer_name_raw in ['center', 'printer_center']:
                printer_short = 'center'
            elif printer_name_raw in ['right', 'printer_right']:
                printer_short = 'right'
            else:
                response.success = False
                response.message = f'未知的打印机: {printer_name_raw}，支持: left/center/right/all'
                self._service_delay(1)
                return response

            success, msg = self._set_single_printer_active(printer_short, active)
            response.success = success
            response.message = msg
            self._service_delay(1)
            return response

    def _set_single_printer_active(self, printer_short: str, active: bool):
        """
        设置单个打印机的激活状态

        Args:
            printer_short: 打印机短名称 (left/center/right)
            active: 是否激活（允许发送指令）

        Returns:
            (成功标志, 消息) 元组
        """
        printer_name = f'printer_{printer_short}'
        param_name = f'{printer_name}_enabled'

        # 1. 更新 ROS 2 参数（会触发参数回调）
        try:
            self.set_parameters([Parameter(param_name, Parameter.Type.BOOL, active)])
            self.get_logger().info(f'已更新参数: {param_name} = {active}')
        except Exception as e:
            msg = f'更新参数失败: {str(e)}'
            self.get_logger().error(msg)
            return False, msg

        # 2. 持久化到 yaml 文件
        try:
            self._persist_enabled_to_yaml(printer_name, active)
            msg = f'[{printer_name}] 激活状态已设置为 {active} 并持久化'
            self.get_logger().info(msg)
            return True, msg
        except Exception as e:
            msg = f'[{printer_name}] 激活状态已设置为 {active}，但持久化失败: {str(e)}'
            self.get_logger().warning(msg)
            return True, msg  # 即使持久化失败，参数已更新，仍返回成功

    def _persist_enabled_to_yaml(self, printer_name: str, enabled: bool) -> None:
        """
        持久化 enabled 参数到 yaml 文件

        Args:
            printer_name: 打印机名称 (printer_left/printer_center/printer_right)
            enabled: 是否允许发送指令
        """
        try:
            # 读取现有配置
            with self._config_path.open('r', encoding='utf-8') as f:
                config = yaml.safe_load(f) or {}

            # 更新 enabled 字段
            if 'connections' in config and printer_name in config['connections']:
                config['connections'][printer_name]['enabled'] = enabled
                self.get_logger().debug(
                    f'更新配置: connections.{printer_name}.enabled = {enabled}'
                )
            else:
                raise ValueError(f'配置文件中未找到 {printer_name}')

            # 写回文件
            with self._config_path.open('w', encoding='utf-8') as f:
                yaml.dump(
                    config,
                    f,
                    default_flow_style=False,
                    allow_unicode=True,
                    sort_keys=False
                )

            self.get_logger().info(f'已持久化 {printer_name}.enabled = {enabled} 到 {self._config_path}')

        except Exception as e:
            self.get_logger().error(f'持久化到 yaml 失败: {e}')
            raise

    def destroy_node(self) -> bool:
        """优雅关闭节点"""
        self.get_logger().info('正在关闭节点...')

        for name, client in self._tcp_clients.items():
            future = asyncio.run_coroutine_threadsafe(client.stop(), self._loop)
            try:
                future.result(timeout=5.0)
            except Exception as e:
                self.get_logger().error(f'停止 {name} 失败: {e}')

        self._shutdown_event.set()
        if self._async_thread and self._async_thread.is_alive():
            self._async_thread.join(timeout=5.0)

        self.get_logger().info('节点已关闭')
        return super().destroy_node()


def main(args=None) -> None:
    """主函数"""
    rclpy.init(args=args)
    node = AsyncInkjetPrinterNode()
    executor = MultiThreadedExecutor(num_threads=10)
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
