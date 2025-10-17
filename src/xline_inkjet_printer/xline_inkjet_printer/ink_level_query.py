"""
墨盒模量查询模块

通过独立的8010端口查询打印机墨盒模量。
采用按需连接方式，查询时建立连接，查询完成后关闭。
"""

import socket
import asyncio
from typing import Optional


class InkLevelQuery:
    """
    墨盒模量查询器

    协议说明：
    - 发送指令：1B 02 00 26 01 1B 03 9E
    - 响应格式：1B 06 00 12 26 [XX] 1B 03 27
    - 模量位置：响应的第6个字节（索引5）
    """

    # 固定查询指令
    QUERY_COMMAND = bytes.fromhex('1B 02 00 26 01 1B 03 9E')

    # 响应帧头
    RESPONSE_HEADER = bytes.fromhex('1B 06 00 12 26')

    # 响应长度
    RESPONSE_LENGTH = 8  # 1B 06 00 12 26 [XX] 1B 03 27

    # 模量数据在响应中的索引
    INK_LEVEL_INDEX = 5

    def __init__(self, host: str, port: int = 8010, timeout: float = 3.0):
        """
        初始化墨盒查询器

        Args:
            host: 打印机IP地址
            port: 查询端口（默认8010）
            timeout: 超时时间（秒）
        """
        self.host = host
        self.port = port
        self.timeout = timeout

    async def query_ink_level(self) -> Optional[int]:
        """
        查询墨盒模量（异步版本）

        Returns:
            墨盒模量值（0-255），查询失败返回None

        Examples:
            >>> query = InkLevelQuery('192.168.1.100')
            >>> level = await query.query_ink_level()
            >>> print(f"墨盒模量: {level}")
        """
        try:
            # 创建连接
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(self.host, self.port),
                timeout=self.timeout
            )

            try:
                # 发送查询指令
                writer.write(self.QUERY_COMMAND)
                await writer.drain()

                # 接收响应
                response = await asyncio.wait_for(
                    reader.read(self.RESPONSE_LENGTH),
                    timeout=self.timeout
                )

                # 验证响应
                if len(response) < self.RESPONSE_LENGTH:
                    return None

                if not response.startswith(self.RESPONSE_HEADER):
                    return None

                # 提取模量值
                ink_level = response[self.INK_LEVEL_INDEX]
                return ink_level

            finally:
                # 关闭连接
                writer.close()
                await writer.wait_closed()

        except (asyncio.TimeoutError, ConnectionRefusedError, OSError) as e:
            return None

    def query_ink_level_sync(self) -> Optional[int]:
        """
        查询墨盒模量（同步版本）

        Returns:
            墨盒模量值（0-255），查询失败返回None

        Examples:
            >>> query = InkLevelQuery('192.168.1.100')
            >>> level = query.query_ink_level_sync()
            >>> print(f"墨盒模量: {level}")
        """
        try:
            # 创建socket连接
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)

            try:
                # 连接
                sock.connect((self.host, self.port))

                # 发送查询指令
                sock.sendall(self.QUERY_COMMAND)

                # 接收响应
                response = sock.recv(self.RESPONSE_LENGTH)

                # 验证响应
                if len(response) < self.RESPONSE_LENGTH:
                    return None

                if not response.startswith(self.RESPONSE_HEADER):
                    return None

                # 提取模量值
                ink_level = response[self.INK_LEVEL_INDEX]
                return ink_level

            finally:
                sock.close()

        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            return None


# 便捷函数
async def query_ink_level_async(host: str, port: int = 8010, timeout: float = 3.0) -> Optional[int]:
    """
    查询墨盒模量（异步便捷函数）

    Args:
        host: 打印机IP地址
        port: 查询端口
        timeout: 超时时间

    Returns:
        墨盒模量值，失败返回None
    """
    query = InkLevelQuery(host, port, timeout)
    return await query.query_ink_level()


def query_ink_level_sync(host: str, port: int = 8010, timeout: float = 3.0) -> Optional[int]:
    """
    查询墨盒模量（同步便捷函数）

    Args:
        host: 打印机IP地址
        port: 查询端口
        timeout: 超时时间

    Returns:
        墨盒模量值，失败返回None
    """
    query = InkLevelQuery(host, port, timeout)
    return query.query_ink_level_sync()
