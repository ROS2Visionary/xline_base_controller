import socket
import threading
import select
from pathlib import Path
from typing import Optional, Dict, Any

import yaml


class TcpClient:
    """
    可独立管理的TCP客户端：支持从包内相对路径YAML读取参数、断线重连、被动断开检测、热更新。

    支持两种YAML结构：
    1) 单连接(向后兼容)：
        ip, port, timeout, reconnect_times, reconnect_interval
    2) 合并配置(推荐)：
        global_defaults: { timeout, reconnect_times, reconnect_interval }
        connections:
          <section>:
            ip, port, [timeout, reconnect_times, reconnect_interval, enabled]
    """

    def __init__(self, logger, config_name: str = 'printer.yaml', name: Optional[str] = None, section: Optional[str] = None) -> None:
        self._logger = logger
        self._name = name or 'tcp'
        self._section = section  # 在合并配置中使用的连接名

        # 运行参数（从YAML加载）
        self._ip: str = '127.0.0.1'
        self._port: int = 9100
        self._connect_timeout: float = 3.0
        self._max_reconnect_attempts: int = 0  # 0 表示无限重连
        self._reconnect_interval: float = 2.0

        # TCP 连接相关
        self._sock: Optional[socket.socket] = None
        self._connected: bool = False
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._reconnect_attempts_done: int = 0
        self._gave_up: bool = False
        self._enabled: bool = True

        # 配置文件路径（相对：包内 config/<config_name>）
        self._config_path = Path(__file__).resolve().parent / 'config' / config_name

        # 后台连接管理线程
        self._conn_thread = threading.Thread(target=self._connection_manager, daemon=True)

        # 启动时先加载一次配置
        self.updateParameter()

    # 使用相对路径从包内config读取参数
    def updateParameter(self) -> None:
        try:
            with self._config_path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            # 如果是合并配置（包含connections）且指定了section，则按section读取
            if self._section and isinstance(data, dict) and 'connections' in data:
                self._update_from_merged_yaml(data)
            else:
                self._update_from_single_yaml(data)
        except Exception as e:
            self._logger.warn(f'[{self._name}] 读取配置失败，沿用旧参数: {e}')

    def _update_from_merged_yaml(self, data: Dict[str, Any]) -> None:
        conns = data.get('connections', {}) or {}
        if self._section not in conns:
            self._logger.warn(f'[{self._name}] 未找到section={self._section}，跳过本次更新')
            return
        defaults = data.get('global_defaults', {}) or {}
        entry = conns.get(self._section, {}) or {}

        # merge: entry覆盖defaults
        def gv(key: str, fallback: Any) -> Any:
            return entry.get(key, defaults.get(key, fallback))

        # 读取字段并校验
        new_ip = str(gv('ip', self._ip))
        new_port = int(gv('port', self._port))

        # timeout
        try:
            new_timeout = float(gv('timeout', self._connect_timeout))
        except Exception:
            new_timeout = self._connect_timeout
        if new_timeout <= 0:
            new_timeout = self._connect_timeout

        # reconnect_times
        try:
            new_max_reconnect = int(gv('reconnect_times', self._max_reconnect_attempts))
        except Exception:
            new_max_reconnect = self._max_reconnect_attempts
        if new_max_reconnect < 0:
            new_max_reconnect = 0

        # reconnect_interval
        try:
            new_reconnect_interval = float(gv('reconnect_interval', self._reconnect_interval))
        except Exception:
            new_reconnect_interval = self._reconnect_interval
        if new_reconnect_interval <= 0:
            new_reconnect_interval = self._reconnect_interval

        # 是否启用（默认启用）
        enabled = bool(entry.get('enabled', True))
        if enabled != self._enabled:
            self._enabled = enabled
            if not self._enabled:
                if self._connected:
                    self._logger.info(f'[{self._name}] 配置禁用，关闭连接')
                self._reset_connection()
                self._reset_attempts()
                return
            else:
                self._logger.info(f'[{self._name}] 配置启用，恢复连接管理')
                self._reset_attempts()

        self._apply_new_params(new_ip, new_port, new_timeout, new_max_reconnect, new_reconnect_interval)

    def _update_from_single_yaml(self, data: Dict[str, Any]) -> None:
        new_ip = str(data.get('ip', self._ip))
        new_port = int(data.get('port', self._port))

        # 读取新参数: 超时与最大重连次数
        new_timeout_raw = data.get('timeout', self._connect_timeout)
        try:
            new_timeout = float(new_timeout_raw)
        except Exception:
            new_timeout = self._connect_timeout
        if new_timeout <= 0:
            new_timeout = self._connect_timeout

        new_max_reconnect = int(data.get('reconnect_times', self._max_reconnect_attempts))
        if new_max_reconnect < 0:
            new_max_reconnect = 0  # 负数按0(无限重连)

        new_reconnect_interval_raw = data.get('reconnect_interval', self._reconnect_interval)
        try:
            new_reconnect_interval = float(new_reconnect_interval_raw)
        except Exception:
            new_reconnect_interval = self._reconnect_interval
        if new_reconnect_interval <= 0:
            new_reconnect_interval = self._reconnect_interval

        # 顶层可选 enabled（兼容单文件启停）
        enabled = bool(data.get('enabled', self._enabled))
        if enabled != self._enabled:
            self._enabled = enabled
            if not self._enabled:
                if self._connected:
                    self._logger.info(f'[{self._name}] 配置禁用，关闭连接')
                self._reset_connection()
                self._reset_attempts()
                return
            else:
                self._logger.info(f'[{self._name}] 配置启用，恢复连接管理')
                self._reset_attempts()

        self._apply_new_params(new_ip, new_port, new_timeout, new_max_reconnect, new_reconnect_interval)

    def _apply_new_params(self, new_ip: str, new_port: int, new_timeout: float, new_max_reconnect: int, new_reconnect_interval: float) -> None:
        # 配置变更处理
        if new_ip != self._ip or new_port != self._port:
            old = f"{self._ip}:{self._port}"
            self._ip, self._port = new_ip, new_port
            self._logger.info(f'[{self._name}] 加载新地址: {old} -> {self._ip}:{self._port}')
            self._reset_connection()
            self._reset_attempts()

        updated_misc = False
        if new_timeout != self._connect_timeout:
            self._connect_timeout = new_timeout
            updated_misc = True
            self._logger.info(f'[{self._name}] 更新连接超时: {self._connect_timeout}s')
        if new_max_reconnect != self._max_reconnect_attempts:
            self._max_reconnect_attempts = new_max_reconnect
            updated_misc = True
            tip = '无限' if self._max_reconnect_attempts == 0 else str(self._max_reconnect_attempts)
            self._logger.info(f'[{self._name}] 更新最大重连次数: {tip}')
        if new_reconnect_interval != self._reconnect_interval:
            self._reconnect_interval = new_reconnect_interval
            updated_misc = True
            self._logger.info(f'[{self._name}] 更新重连间隔: {self._reconnect_interval}s')
        if updated_misc:
            self._reset_attempts()

    def start(self) -> None:
        # disabled 时不启动连接管理线程
        if not self._enabled:
            self._logger.debug(f'[{self._name}] 已禁用，跳过启动连接管理线程')
            return
        if not self._conn_thread.is_alive():
            self._stop_evt.clear()
            self._conn_thread = threading.Thread(target=self._connection_manager, daemon=True)
            self._conn_thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        try:
            if self._conn_thread.is_alive():
                self._conn_thread.join(timeout=2.0)
        except Exception:
            pass
        self._reset_connection()

    def poll(self) -> None:
        self._poll_socket()

    def status(self) -> str:
        return '已连接' if self._connected else '未连接(重连中...)'

    def is_enabled(self) -> bool:
        return self._enabled

    def is_running(self) -> bool:
        return self._conn_thread.is_alive()

    # 内部方法
    def _connection_manager(self) -> None:
        while not self._stop_evt.is_set():
            if not self._enabled:
                self._stop_evt.wait(self._reconnect_interval)
                continue
            if not self._connected:
                # 判断是否还能重连
                if self._max_reconnect_attempts == 0 or self._reconnect_attempts_done < self._max_reconnect_attempts:
                    self._try_connect()
                    self._reconnect_attempts_done += 1
                else:
                    if not self._gave_up:
                        self._gave_up = True
                        self._logger.error(f'[{self._name}] 已达到最大重连次数，停止继续重连。可修改配置或重启以重试。')
            self._stop_evt.wait(self._reconnect_interval)

    def _try_connect(self) -> None:
        with self._lock:
            # 若已有旧连接，先确保关闭
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
                self._connected = False

            try:
                s = socket.create_connection((self._ip, self._port), timeout=self._connect_timeout)
                # 启用keepalive，尽量更快发现断开（具体触发依赖系统）
                try:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                    # Linux可选优化
                    if hasattr(socket, 'TCP_KEEPIDLE'):
                        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
                    if hasattr(socket, 'TCP_KEEPINTVL'):
                        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
                    if hasattr(socket, 'TCP_KEEPCNT'):
                        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                except Exception:
                    pass

                s.setblocking(False)
                self._sock = s
                self._connected = True
                self._reconnect_attempts_done = 0
                self._gave_up = False
                self._logger.info(f'[{self._name}] TCP连接成功 -> {self._ip}:{self._port}')
            except Exception as e:
                self._connected = False
                self._logger.warn(f'[{self._name}] TCP连接失败({self._ip}:{self._port}): {e}')

    def _poll_socket(self) -> None:
        # 利用select+peek判断被动断开
        with self._lock:
            if not self._connected or self._sock is None:
                return
            try:
                r, _, _ = select.select([self._sock], [], [], 0)
                if r:
                    data = self._sock.recv(1, socket.MSG_PEEK)
                    if not data:
                        # 远端关闭
                        self._logger.warn(f'[{self._name}] 检测到TCP被动断开')
                        self._reset_connection()
            except BlockingIOError:
                # 非阻塞且无数据，认为正常
                pass
            except Exception as e:
                self._logger.warn(f'[{self._name}] TCP检测异常，重置连接: {e}')
                self._reset_connection()

    def _reset_connection(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._sock = None
            self._connected = False

    def _reset_attempts(self) -> None:
        # 重置重连计数与放弃标记
        self._reconnect_attempts_done = 0
        self._gave_up = False
