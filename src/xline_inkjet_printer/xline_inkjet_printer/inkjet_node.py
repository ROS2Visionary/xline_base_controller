import socket
import threading
import select
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
import yaml


class InkjetPrinterNode(Node):
    def __init__(self) -> None:
        super().__init__('inkjet_printer_node')

        # 运行参数（从YAML加载）
        self._ip: str = '127.0.0.1'
        self._port: int = 9100
        self._connect_timeout: float = 3.0  # 连接超时(秒)
        self._max_reconnect_attempts: int = 0  # 0 表示无限重连

        # TCP 连接相关
        self._sock: Optional[socket.socket] = None
        self._connected: bool = False
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._reconnect_interval = 2.0  # 秒（可在YAML中配置 reconnect_interval）
        self._reconnect_attempts_done: int = 0
        self._gave_up: bool = False

        # 启动时加载配置
        self.updateParameter()

        # 启动后台连接管理线程
        self._conn_thread = threading.Thread(target=self._connection_manager, daemon=True)
        self._conn_thread.start()

        # 定时上报状态并热加载配置
        self._status_timer = self.create_timer(1.0, self._status_tick)

        self.get_logger().info('inkjet_printer_node 已启动并开始管理TCP连接')

    # 使用相对路径从包内config读取参数
    def updateParameter(self) -> None:
        try:
            cfg_path = Path(__file__).resolve().parent / 'config' / 'printer.yaml'
            with cfg_path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            new_ip = str(data.get('ip', self._ip))
            new_port = int(data.get('port', self._port))
            # 读取新参数: 超时与最大重连次数
            # 超时<=0时回退到旧值
            new_timeout_raw = data.get('timeout', self._connect_timeout)
            try:
                new_timeout = float(new_timeout_raw)
            except Exception:
                new_timeout = self._connect_timeout
            if new_timeout <= 0:
                new_timeout = self._connect_timeout

            # 负数按0(无限重连)
            new_max_reconnect = int(data.get('reconnect_times', self._max_reconnect_attempts))
            if new_max_reconnect < 0:
                new_max_reconnect = 0

            # 重连间隔（秒），<=0 则回退旧值
            new_reconnect_interval_raw = data.get('reconnect_interval', self._reconnect_interval)
            try:
                new_reconnect_interval = float(new_reconnect_interval_raw)
            except Exception:
                new_reconnect_interval = self._reconnect_interval
            if new_reconnect_interval <= 0:
                new_reconnect_interval = self._reconnect_interval

            # 配置变更处理
            if new_ip != self._ip or new_port != self._port:
                old = f"{self._ip}:{self._port}"
                self._ip, self._port = new_ip, new_port
                self.get_logger().info(f'加载新配置: {old} -> {self._ip}:{self._port}')
                # 配置变更则重置连接，触发重连
                self._reset_connection()
                self._reset_attempts()
            # 针对非IP端口的其他参数变化，更新并重置重连计数（不强制断开）
            updated_misc = False
            if new_timeout != self._connect_timeout:
                self._connect_timeout = new_timeout
                updated_misc = True
                self.get_logger().info(f'更新连接超时: {self._connect_timeout}s')
            if new_max_reconnect != self._max_reconnect_attempts:
                self._max_reconnect_attempts = new_max_reconnect
                updated_misc = True
                tip = '无限' if self._max_reconnect_attempts == 0 else str(self._max_reconnect_attempts)
                self.get_logger().info(f'更新最大重连次数: {tip}')
            if new_reconnect_interval != self._reconnect_interval:
                self._reconnect_interval = new_reconnect_interval
                updated_misc = True
                self.get_logger().info(f'更新重连间隔: {self._reconnect_interval}s')
            if updated_misc:
                self._reset_attempts()
        except Exception as e:
            self.get_logger().warn(f'读取配置失败，继续沿用旧参数: {e}')

    def _status_tick(self) -> None:
        # 热加载配置
        self.updateParameter()
        # 检测连接是否被动断开
        self._poll_socket()
        status = '已连接' if self._connected else '未连接(重连中...)'
        self.get_logger().debug(f'TCP状态: {status} -> {self._ip}:{self._port}')

    def _connection_manager(self) -> None:
        while not self._stop_evt.is_set():
            if not self._connected:
                # 判断是否还能重连
                if self._max_reconnect_attempts == 0 or self._reconnect_attempts_done < self._max_reconnect_attempts:
                    self._try_connect()
                    self._reconnect_attempts_done += 1
                else:
                    if not self._gave_up:
                        self._gave_up = True
                        self.get_logger().error('已达到最大重连次数，停止继续重连。可修改配置或重启节点以重试。')
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
                self.get_logger().info(f'TCP连接成功 -> {self._ip}:{self._port}')
            except Exception as e:
                self._connected = False
                self.get_logger().warn(f'TCP连接失败({self._ip}:{self._port}): {e}')

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
                        self.get_logger().warn('检测到TCP被动断开')
                        self._reset_connection()
            except BlockingIOError:
                # 非阻塞且无数据，认为正常
                pass
            except Exception as e:
                self.get_logger().warn(f'TCP检测异常，重置连接: {e}')
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

    def destroy_node(self) -> bool:
        # 优雅关闭
        self._stop_evt.set()
        try:
            if hasattr(self, '_conn_thread') and self._conn_thread.is_alive():
                self._conn_thread.join(timeout=2.0)
        except Exception:
            pass
        self._reset_connection()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InkjetPrinterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
