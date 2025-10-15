from typing import Dict

import rclpy
from rclpy.node import Node

from .tcp_client import TcpClient


class InkjetPrinterNode(Node):
    def __init__(self) -> None:
        super().__init__('inkjet_printer_node')

        # 支持多路独立TCP客户端（避免与rclpy内部 _clients 冲突）
        self._tcp_clients: Dict[str, TcpClient] = {}

        # 创建三路TCP客户端，统一从 printers.yaml 读取指定 section
        self._tcp_clients['printer_left'] = TcpClient(self.get_logger(), config_name='printers.yaml', name='printer_left', section='printer_left')
        self._tcp_clients['printer_center'] = TcpClient(self.get_logger(), config_name='printers.yaml', name='printer_center', section='printer_center')
        self._tcp_clients['printer_right'] = TcpClient(self.get_logger(), config_name='printers.yaml', name='printer_right', section='printer_right')
        for c in self._tcp_clients.values():
            c.start()

        # 定时器：热更新配置与状态检测
        self._status_timer = self.create_timer(1.0, self._status_tick)

        self.get_logger().info('inkjet_printer_node 已启动并开始管理多路TCP连接')

    def _status_tick(self) -> None:
        for name, client in self._tcp_clients.items():
            client.updateParameter()
            client.poll()
            self.get_logger().debug(f'[{name}] TCP状态: {client.status()}')

    def destroy_node(self) -> bool:
        # 优雅关闭全部TCP客户端
        try:
            for client in self._tcp_clients.values():
                client.stop()
        except Exception:
            pass
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
