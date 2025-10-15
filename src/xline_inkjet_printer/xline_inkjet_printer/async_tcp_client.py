"""
基于 asyncio 的异步 TCP 客户端

提供高性能、异步的 TCP 连接管理和数据收发功能。
"""

import asyncio
from pathlib import Path
from typing import Optional, Callable, Dict, Any, Awaitable
import yaml

from .protocol import InkjetProtocol, InkjetCommand
from .config_validator import ConfigValidator, ValidationError


class AsyncTcpClient:
    """
    异步 TCP 客户端

    特性：
    - 基于 asyncio，高性能异步I/O
    - 自动重连机制
    - 协议解析和编码
    """

    def __init__(
        self,
        logger,
        config_name: str = 'printers.yaml',
        name: str = 'tcp',
        section: Optional[str] = None,
        device_id: int = 0
    ) -> None:
        """
        初始化异步 TCP 客户端

        Args:
            logger: ROS2 logger 对象
            config_name: 配置文件名
            name: 客户端名称
            section: 配置section名称（用于多连接配置）
            device_id: 打印机设备号（0-255）
        """
        self._logger = logger
        self._name = name
        self._section = section
        self._device_id = device_id
        self._config_path = Path(__file__).resolve().parent / 'config' / config_name

        # 连接参数
        self._ip: str = '127.0.0.1'
        self._port: int = 9100
        self._connect_timeout: float = 3.0
        self._max_reconnect_attempts: int = 0  # 0 = 无限重连
        self._reconnect_interval: float = 2.0
        self._enabled: bool = True

        # asyncio 组件
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._connected: bool = False

        # 后台任务
        self._reconnect_task: Optional[asyncio.Task] = None
        self._recv_task: Optional[asyncio.Task] = None

        # 重连状态
        self._reconnect_attempts_done: int = 0
        self._gave_up: bool = False
        self._stop_event: asyncio.Event = asyncio.Event()

        # 协议层
        self._protocol = InkjetProtocol()
        self._logger.info(f'[{self._name}] 协议已初始化，设备号={self._device_id}')

        # 回调函数
        self._on_frame: Optional[Callable[[Dict[str, Any]], Awaitable[None]]] = None
        self._on_state_change: Optional[Callable[[bool], Awaitable[None]]] = None

        # 加载配置（仅一次）
        self._load_config()

    def _load_config(self) -> None:
        """加载配置（仅在初始化时调用一次）"""
        try:
            with self._config_path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}

            if self._section and 'connections' in data:
                self._load_from_merged_config(data)
            else:
                self._load_from_single_config(data)

            self._logger.info(
                f'[{self._name}] 配置加载成功: '
                f'{self._ip}:{self._port}, '
                f'enabled={self._enabled}'
            )

        except Exception as e:
            self._logger.error(f'[{self._name}] 加载配置失败: {e}，使用默认值')

    def _load_from_merged_config(self, data: Dict[str, Any]) -> None:
        """从合并配置加载参数"""
        conns = data.get('connections', {}) or {}
        if self._section not in conns:
            self._logger.warning(f'[{self._name}] 配置中未找到 section={self._section}')
            return

        defaults = data.get('global_defaults', {}) or {}
        entry = conns.get(self._section, {}) or {}

        # 合并配置（entry 覆盖 defaults）
        merged = {**defaults, **entry}

        # 验证并应用配置
        try:
            validated = ConfigValidator.validate_connection_config(merged, self._section)
            self._apply_config(validated)
        except ValidationError as e:
            self._logger.error(f'[{self._name}] 配置验证失败: {e}')

    def _load_from_single_config(self, data: Dict[str, Any]) -> None:
        """从单一配置加载参数"""
        try:
            validated = ConfigValidator.validate_connection_config(data, self._name)
            self._apply_config(validated)
        except ValidationError as e:
            self._logger.error(f'[{self._name}] 配置验证失败: {e}')

    def _apply_config(self, config: Dict[str, Any]) -> None:
        """应用配置参数"""
        if 'ip' in config:
            self._ip = config['ip']

        if 'port' in config:
            self._port = config['port']

        if 'timeout' in config:
            self._connect_timeout = config['timeout']

        if 'reconnect_times' in config:
            self._max_reconnect_attempts = config['reconnect_times']

        if 'reconnect_interval' in config:
            self._reconnect_interval = config['reconnect_interval']

        if 'enabled' in config:
            self._enabled = config['enabled']

    async def connect(self) -> bool:
        """
        建立 TCP 连接

        Returns:
            是否成功连接
        """
        if self._connected:
            return True

        try:
            self._reader, self._writer = await asyncio.wait_for(
                asyncio.open_connection(self._ip, self._port),
                timeout=self._connect_timeout
            )

            self._connected = True
            self._reconnect_attempts_done = 0
            self._gave_up = False

            self._logger.info(f'[{self._name}] 连接成功 -> {self._ip}:{self._port}')

            # 触发状态变化回调
            if self._on_state_change:
                try:
                    await self._on_state_change(True)
                except Exception as e:
                    self._logger.error(f'[{self._name}] 状态回调异常: {e}')

            # 启动接收循环
            if self._recv_task is None or self._recv_task.done():
                self._recv_task = asyncio.create_task(self._receive_loop())

            return True

        except asyncio.TimeoutError:
            self._logger.warning(f'[{self._name}] 连接超时: {self._ip}:{self._port}')
            return False

        except (OSError, ConnectionError) as e:
            self._logger.warning(f'[{self._name}] 连接失败: {e}')
            return False

        except Exception as e:
            self._logger.error(f'[{self._name}] 连接异常: {e}')
            return False

    async def _disconnect(self) -> None:
        """断开连接"""
        if not self._connected:
            return

        self._connected = False

        # 关闭 writer
        if self._writer:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except Exception as e:
                self._logger.debug(f'[{self._name}] 关闭连接异常: {e}')
            finally:
                self._writer = None
                self._reader = None

        self._logger.info(f'[{self._name}] 已断开连接')

        # 触发状态变化回调
        if self._on_state_change:
            try:
                await self._on_state_change(False)
            except Exception as e:
                self._logger.error(f'[{self._name}] 状态回调异常: {e}')

    async def send_raw(self, data: bytes) -> bool:
        """
        发送原始字节数据

        Args:
            data: 要发送的字节流

        Returns:
            是否发送成功
        """
        if not self._connected or not self._writer:
            self._logger.warning(f'[{self._name}] 未连接，无法发送数据')
            return False

        try:
            self._writer.write(data)
            await self._writer.drain()

            self._logger.debug(f'[{self._name}] 发送 {len(data)} 字节: {data.hex()}')
            return True

        except (OSError, ConnectionError) as e:
            self._logger.error(f'[{self._name}] 发送失败: {e}')
            await self._disconnect()
            return False

        except Exception as e:
            self._logger.error(f'[{self._name}] 发送异常: {e}')
            return False

    async def send_command(
        self,
        command_code: int,
        json_data: Dict[str, Any]
    ) -> bool:
        """
        发送协议命令

        Args:
            command_code: 指令码（InkjetCommand枚举或整数）
            json_data: JSON数据字典

        Returns:
            是否发送成功

        Examples:
            >>> await client.send_command(InkjetCommand.NOISES, {"EU2L": {"noises": 1}})
        """
        try:
            data = self._protocol.encode_command(
                self._device_id,
                command_code,
                json_data
            )
            return await self.send_raw(data)
        except Exception as e:
            self._logger.error(f'[{self._name}] 编码命令失败: {e}')
            return False

    async def _receive_loop(self) -> None:
        """接收循环（后台任务）"""
        self._logger.debug(f'[{self._name}] 接收循环已启动')

        try:
            while self._connected and self._reader and not self._stop_event.is_set():
                try:
                    # 读取数据
                    data = await asyncio.wait_for(
                        self._reader.read(4096),
                        timeout=1.0  # 1秒超时，允许检查停止事件
                    )

                    if not data:
                        # 连接被远端关闭
                        self._logger.warning(f'[{self._name}] 检测到远端关闭连接')
                        await self._disconnect()
                        break

                    self._logger.debug(f'[{self._name}] 接收 {len(data)} 字节: {data.hex()}')

                    # 协议解析（feed_data 可能返回多个完整帧）
                    frames = self._protocol.feed_data(data)

                    for frame in frames:
                        self._logger.info(
                            f'[{self._name}] 解析帧: '
                            f'设备={frame["device_id"]}, '
                            f'指令={frame["command_name"]}(0x{frame["command_code"]:02X}), '
                            f'JSON={frame.get("json_data", "N/A")}'
                        )

                        # 帧回调
                        if self._on_frame:
                            try:
                                await self._on_frame(frame)
                            except Exception as e:
                                self._logger.error(f'[{self._name}] 帧回调异常: {e}')

                except asyncio.TimeoutError:
                    # 1秒超时，继续循环检查
                    continue

                except (OSError, ConnectionError) as e:
                    self._logger.error(f'[{self._name}] 接收异常: {e}')
                    await self._disconnect()
                    break

        except Exception as e:
            self._logger.error(f'[{self._name}] 接收循环异常: {e}')

        finally:
            self._logger.debug(f'[{self._name}] 接收循环已退出')

    async def _reconnect_loop(self) -> None:
        """重连循环（后台任务）"""
        self._logger.debug(f'[{self._name}] 重连循环已启动')

        while not self._stop_event.is_set():
            try:
                # 如果禁用，则等待
                if not self._enabled:
                    await asyncio.sleep(self._reconnect_interval)
                    continue

                # 如果未连接，尝试重连
                if not self._connected:
                    # 检查是否达到最大重连次数
                    if self._max_reconnect_attempts == 0 or \
                       self._reconnect_attempts_done < self._max_reconnect_attempts:

                        if await self.connect():
                            # 连接成功，重置计数
                            self._reconnect_attempts_done = 0
                        else:
                            # 连接失败，增加计数
                            self._reconnect_attempts_done += 1
                    else:
                        # 达到最大重连次数
                        if not self._gave_up:
                            self._gave_up = True
                            self._logger.error(
                                f'[{self._name}] 已达到最大重连次数 '
                                f'({self._max_reconnect_attempts})，停止重连'
                            )

                # 等待重连间隔
                await asyncio.sleep(self._reconnect_interval)

            except Exception as e:
                self._logger.error(f'[{self._name}] 重连循环异常: {e}')
                await asyncio.sleep(self._reconnect_interval)

        self._logger.debug(f'[{self._name}] 重连循环已退出')

    async def start(self) -> None:
        """启动客户端（启动后台任务）"""
        if not self._enabled:
            self._logger.info(f'[{self._name}] 已禁用，跳过启动')
            return

        self._stop_event.clear()

        # 启动重连循环
        if self._reconnect_task is None or self._reconnect_task.done():
            self._reconnect_task = asyncio.create_task(self._reconnect_loop())

        self._logger.info(f'[{self._name}] 客户端已启动')

    async def stop(self) -> None:
        """停止客户端"""
        self._logger.info(f'[{self._name}] 正在停止客户端...')

        self._stop_event.set()

        # 断开连接
        await self._disconnect()

        # 等待任务结束
        tasks = []
        if self._reconnect_task and not self._reconnect_task.done():
            tasks.append(self._reconnect_task)
        if self._recv_task and not self._recv_task.done():
            tasks.append(self._recv_task)

        if tasks:
            await asyncio.wait(tasks, timeout=2.0)

        self._logger.info(f'[{self._name}] 客户端已停止')

    # 回调设置
    def set_frame_callback(self, callback: Callable[[Dict[str, Any]], Awaitable[None]]) -> None:
        """设置帧接收回调"""
        self._on_frame = callback

    def set_state_callback(self, callback: Callable[[bool], Awaitable[None]]) -> None:
        """设置连接状态变化回调"""
        self._on_state_change = callback

    # 状态查询
    def is_connected(self) -> bool:
        """是否已连接"""
        return self._connected

    def is_enabled(self) -> bool:
        """是否已启用"""
        return self._enabled

    def get_status(self) -> str:
        """获取状态字符串"""
        if not self._enabled:
            return '已禁用'
        elif self._connected:
            return '已连接'
        elif self._gave_up:
            return '已放弃重连'
        else:
            return f'重连中 ({self._reconnect_attempts_done}/{self._max_reconnect_attempts or "∞"})'

    def get_device_id(self) -> int:
        """获取设备号"""
        return self._device_id
