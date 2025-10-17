"""
异步喷墨打印机 ROS 2 节点

基于 asyncio 的高性能打印机控制节点。
"""

import asyncio
import threading
from typing import Dict, Optional
import json
from pathlib import Path
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from std_msgs.msg import String
from xline_msgs.srv import PrinterCommand, QuickCommand, SetPrinterEnabled

from .async_tcp_client import AsyncTcpClient
from .protocol import InkjetCommand
from .command_templates import PrinterCommandTemplates
from .ink_level_query import InkLevelQuery


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

        # 声明 enabled 参数（从配置文件读取初始值）
        self.declare_parameter('printer_left_enabled', True)
        self.declare_parameter('printer_center_enabled', True)
        self.declare_parameter('printer_right_enabled', True)

        # 获取参数
        config_file = self.get_parameter('config_file').value
        status_rate = self.get_parameter('status_publish_rate').value
        device_id_left = self.get_parameter('device_id_left').value
        device_id_center = self.get_parameter('device_id_center').value
        device_id_right = self.get_parameter('device_id_right').value

        # 配置文件路径（用于持久化）
        self._config_path = Path(__file__).resolve().parent / 'config' / config_file

        # TCP 客户端字典
        self._tcp_clients: Dict[str, AsyncTcpClient] = {}

        # 墨盒查询器字典
        self._ink_queries: Dict[str, InkLevelQuery] = {}

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

            # 创建墨盒查询器（使用相同IP，端口8010）
            ip = client._ip
            self._ink_queries[name] = InkLevelQuery(host=ip, port=8010, timeout=3.0)

        # ROS 2 发布者
        self._status_pub = self.create_publisher(String, 'printer_status', 10)

        # ROS 2 服务 - 发送命令
        # 通用服务（使用 printer_name 字段路由）
        self._send_generic_srv = self.create_service(
            PrinterCommand, 'printer/send_command', self._handle_send_command_generic
        )

        # ROS 2 服务 - 便捷指令（统一服务，支持所有打印机和常用动作）
        self._quick_command_srv = self.create_service(
            QuickCommand, 'printer/quick_command', self._handle_quick_command
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

        # ROS 2 服务 - 设置打印机启用状态
        self._set_enabled_srv = self.create_service(
            SetPrinterEnabled, 'printer/set_enabled', self._handle_set_enabled
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

        # 注册参数回调
        self.add_on_set_parameters_callback(self._parameters_callback)

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

    # 便捷服务处理函数 - 统一快速命令
    def _handle_quick_command(self, request, response):
        """
        处理快速命令请求

        支持的动作: beep, start_print, stop_print, clean_nozzle, ink_level
        支持的打印机: left, center, right, all
        """
        action = request.action.lower().strip()
        printer_name_raw = request.printer_name.lower().strip()
        param = request.param if request.param > 0 else None

        # 墨盒查询特殊处理（不使用模板）
        if action in ['ink_level', 'query_ink', 'ink']:
            return self._handle_ink_level_query(printer_name_raw, response)

        # 解析动作
        action_map = {
            'beep': ('蜂鸣', lambda p: PrinterCommandTemplates.beep(p if p else 1)),
            'start_print': ('开启打印', lambda p: PrinterCommandTemplates.start_print()),
            'start': ('开启打印', lambda p: PrinterCommandTemplates.start_print()),
            'stop_print': ('关闭打印', lambda p: PrinterCommandTemplates.stop_print()),
            'stop': ('关闭打印', lambda p: PrinterCommandTemplates.stop_print()),
            'clean_nozzle': ('清洗喷头', lambda p: PrinterCommandTemplates.clean_nozzle(p if p else 20)),
            'clean': ('清洗喷头', lambda p: PrinterCommandTemplates.clean_nozzle(p if p else 20)),
        }

        if action not in action_map:
            response.success = False
            response.message = f'不支持的动作: {action}，支持: {", ".join(list(action_map.keys()) + ["ink_level"])}'
            return response

        action_name, template_func = action_map[action]

        # 解析打印机名称
        if printer_name_raw == 'all':
            # 对所有打印机执行
            results = []
            for pname in ['printer_left', 'printer_center', 'printer_right']:
                try:
                    command_code, json_data = template_func(param)
                    # 创建临时响应对象
                    temp_response = QuickCommand.Response()
                    self._execute_template_command(pname, command_code, json_data, action_name, temp_response)
                    results.append(f'{pname}: {temp_response.message}')
                except Exception as e:
                    results.append(f'{pname}: 错误 - {str(e)}')

            response.success = True
            response.message = '\n'.join(results)
            return response
        else:
            # 单个打印机
            # 规范化名称
            if printer_name_raw in ['left', 'printer_left']:
                printer_name = 'printer_left'
            elif printer_name_raw in ['center', 'printer_center']:
                printer_name = 'printer_center'
            elif printer_name_raw in ['right', 'printer_right']:
                printer_name = 'printer_right'
            else:
                response.success = False
                response.message = f'未知的打印机: {printer_name_raw}，支持: left/center/right/all'
                return response

            try:
                command_code, json_data = template_func(param)
                return self._execute_template_command(printer_name, command_code, json_data, action_name, response)
            except Exception as e:
                response.success = False
                response.message = f'执行失败: {str(e)}'
                return response

    def _handle_ink_level_query(self, printer_name_raw: str, response):
        """
        处理墨盒模量查询

        Args:
            printer_name_raw: 打印机名称（left/center/right/all）
            response: 响应对象

        Returns:
            填充后的响应对象
        """
        # 解析打印机名称
        if printer_name_raw == 'all':
            # 查询所有打印机
            results = []
            for pname in ['printer_left', 'printer_center', 'printer_right']:
                query = self._ink_queries.get(pname)
                if not query:
                    results.append(f'{pname}: 查询器未初始化')
                    continue

                # 在 asyncio 循环中执行查询
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
            return response
        else:
            # 单个打印机
            # 规范化名称
            if printer_name_raw in ['left', 'printer_left']:
                printer_name = 'printer_left'
            elif printer_name_raw in ['center', 'printer_center']:
                printer_name = 'printer_center'
            elif printer_name_raw in ['right', 'printer_right']:
                printer_name = 'printer_right'
            else:
                response.success = False
                response.message = f'未知的打印机: {printer_name_raw}，支持: left/center/right/all'
                return response

            query = self._ink_queries.get(printer_name)
            if not query:
                response.success = False
                response.message = f'{printer_name} 查询器未初始化'
                return response

            # 在 asyncio 循环中执行查询
            future = asyncio.run_coroutine_threadsafe(
                query.query_ink_level(),
                self._loop
            )

            try:
                ink_level = future.result(timeout=3.0)
                if ink_level is not None:
                    response.success = True
                    response.message = f'[{printer_name}] 墨盒模量: {ink_level} (0x{ink_level:02X})'
                    self.get_logger().info(response.message)
                else:
                    response.success = False
                    response.message = f'[{printer_name}] 墨盒模量查询失败'
                    self.get_logger().warn(response.message)
            except Exception as e:
                response.success = False
                response.message = f'[{printer_name}] 墨盒模量查询异常: {str(e)}'
                self.get_logger().error(response.message)

            return response

    def _execute_template_command(self, printer_name: str, command_code: int, json_data: dict, action_name: str, response):
        """
        执行模板命令的通用处理函数

        Args:
            printer_name: 打印机名称
            command_code: 指令码
            json_data: JSON数据
            action_name: 动作名称（用于日志）
            response: 响应对象

        Returns:
            填充后的响应对象
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
                response.message = f'[{printer_name}] {action_name}命令已发送: {cmd_name}(0x{command_code:02X})'
                self.get_logger().info(response.message)
            else:
                response.message = f'[{printer_name}] {action_name}命令发送失败'
                self.get_logger().warn(response.message)

        except Exception as e:
            response.success = False
            response.message = f'[{printer_name}] {action_name}命令异常: {str(e)}'
            self.get_logger().error(response.message)

        return response

    # ========== 参数回调 ==========
    def _parameters_callback(self, params):
        """
        参数变化回调函数

        当参数通过 ros2 param set 或服务修改时触发
        """
        successful = True
        for param in params:
            if param.name in ['printer_left_enabled', 'printer_center_enabled', 'printer_right_enabled']:
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

        from rclpy.parameter import SetParametersResult
        return SetParametersResult(successful=successful)

    # ========== 服务处理函数 - 设置打印机启用状态 ==========
    def _handle_set_enabled(self, request, response):
        """
        处理设置打印机启用状态的服务请求

        流程：
        1. 更新 ROS 2 参数（触发参数回调 -> 更新内存 -> 控制连接）
        2. 持久化到 yaml 文件
        """
        printer_name_raw = request.printer_name.lower().strip()
        enabled = request.enabled

        self.get_logger().info(
            f'收到设置启用状态请求: printer={printer_name_raw}, enabled={enabled}'
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
                return response

            success, msg = self._set_single_printer_enabled(printer_short, enabled)
            response.success = success
            response.message = msg
            return response

    def _set_single_printer_enabled(self, printer_short: str, enabled: bool):
        """
        设置单个打印机的启用状态

        Args:
            printer_short: 打印机短名称 (left/center/right)
            enabled: 是否启用

        Returns:
            (成功标志, 消息) 元组
        """
        printer_name = f'printer_{printer_short}'
        param_name = f'{printer_name}_enabled'

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
            self._persist_enabled_to_yaml(printer_name, enabled)
            msg = f'[{printer_name}] 启用状态已设置为 {enabled} 并持久化'
            self.get_logger().info(msg)
            return True, msg
        except Exception as e:
            msg = f'[{printer_name}] 启用状态已设置为 {enabled}，但持久化失败: {str(e)}'
            self.get_logger().warning(msg)
            return True, msg  # 即使持久化失败，参数已更新，仍返回成功

    def _persist_enabled_to_yaml(self, printer_name: str, enabled: bool) -> None:
        """
        持久化 enabled 参数到 yaml 文件

        Args:
            printer_name: 打印机名称 (printer_left/printer_center/printer_right)
            enabled: 是否启用
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
