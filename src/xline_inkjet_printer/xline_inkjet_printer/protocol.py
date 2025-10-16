"""
喷墨打印机协议层

基于实际硬件协议设计，支持编码/解码打印机命令。
"""

from typing import Dict, Any, Optional, Tuple
import json
import struct
from enum import IntEnum


class InkjetCommand(IntEnum):
    """
    喷墨打印机指令码枚举

    根据实际硬件协议定义，每个指令对应一个字节的指令码。
    """
    NOISES = 0x15           # 蜂鸣
    SETUP_EVENT = 0x19      # 启动/关闭打印




class InkjetProtocol:
    """
    喷墨打印机通信协议

    协议格式：
    ┌─────────┬──────────┬──────────┬────────────────┐
    │ 帧头    │ 设备号   │ 指令码   │ JSON数据       │
    │ 2 bytes │ 1 byte   │ 1 byte   │ variable bytes │
    │ 0x1B02  │ 0x00-FF  │ 0x00-FF  │ ASCII JSON     │
    └─────────┴──────────┴──────────┴────────────────┘

    实际示例：
    1b 02 00 15 7b224555324c223a7b226e6f69736573223a317d7d
    ↓ 解析为
    - 帧头: 0x1B 0x02
    - 设备号: 0x00
    - 指令码: 0x15 (NOISES)
    - JSON: {"EU2L":{"noises":1}}
    """

    # 协议常量
    FRAME_HEADER = b'\x1b\x02'      # 固定帧头
    HEADER_LENGTH = 2                # 帧头长度
    DEVICE_ID_LENGTH = 1             # 设备号长度
    COMMAND_LENGTH = 1               # 指令码长度
    HEADER_TOTAL_LENGTH = 4          # 总头部长度（帧头+设备号+指令码）

    def __init__(self):
        """初始化协议"""
        self._buffer = bytearray()   # 接收缓冲区（用于处理TCP分包）

    def encode_command(
        self,
        device_id: int,
        command_code: int,
        json_data: Dict[str, Any]
    ) -> bytes:
        """
        编码命令为字节流

        Args:
            device_id: 设备号 (0-255)
            command_code: 指令码 (0-255 或 InkjetCommand 枚举)
            json_data: JSON数据字典

        Returns:
            编码后的字节流

        Raises:
            ValueError: 如果参数无效

        Examples:
            >>> protocol = InkjetProtocol()
            >>> data = {"EU2L": {"noises": 1}}
            >>> result = protocol.encode_command(0, 0x15, data)
            >>> result.hex()
            '1b0200157b224555324c223a7b226e6f69736573223a317d7d'
        """
        # 验证参数
        if not (0 <= device_id <= 255):
            raise ValueError(f"设备号必须在 0-255 范围内，当前值: {device_id}")

        if not (0 <= command_code <= 255):
            raise ValueError(f"指令码必须在 0-255 范围内，当前值: {command_code}")

        if not isinstance(json_data, dict):
            raise ValueError("json_data 必须是字典类型")

        # 构建帧
        frame = bytearray()

        # 1. 添加帧头
        frame.extend(self.FRAME_HEADER)

        # 2. 添加设备号
        frame.append(device_id)

        # 3. 添加指令码
        frame.append(command_code)

        # 4. 添加JSON数据（转为紧凑格式，不带空格）
        try:
            json_str = json.dumps(json_data, separators=(',', ':'), ensure_ascii=True)
            frame.extend(json_str.encode('ascii'))
        except Exception as e:
            raise ValueError(f"JSON编码失败: {e}")

        return bytes(frame)

    def decode_frame(self, data: bytes) -> Optional[Dict[str, Any]]:
        """
        解码单个完整帧

        Args:
            data: 字节流数据

        Returns:
            解码后的字典，包含:
            - device_id: 设备号
            - command_code: 指令码
            - command_name: 指令名称（如果已知）
            - json_data: JSON数据字典
            - raw_json: 原始JSON字符串

            如果数据不完整或无效则返回 None

        Examples:
            >>> protocol = InkjetProtocol()
            >>> raw = bytes.fromhex('1b0200157b224555324c223a7b226e6f69736573223a317d7d')
            >>> result = protocol.decode_frame(raw)
            >>> result['device_id']
            0
            >>> result['command_code']
            21
            >>> result['json_data']
            {'EU2L': {'noises': 1}}
        """
        # 检查最小长度
        if len(data) < self.HEADER_TOTAL_LENGTH:
            return None

        # 验证帧头
        if data[:self.HEADER_LENGTH] != self.FRAME_HEADER:
            return None

        # 提取设备号
        device_id = data[2]

        # 提取指令码
        command_code = data[3]

        # 提取JSON数据
        json_bytes = data[self.HEADER_TOTAL_LENGTH:]

        if len(json_bytes) == 0:
            # 没有JSON数据（某些指令可能不需要数据）
            return {
                'device_id': device_id,
                'command_code': command_code,
                'command_name': self._get_command_name(command_code),
                'json_data': None,
                'raw_json': ''
            }

        # 解析JSON
        try:
            json_str = json_bytes.decode('ascii')
            json_data = json.loads(json_str)

            return {
                'device_id': device_id,
                'command_code': command_code,
                'command_name': self._get_command_name(command_code),
                'json_data': json_data,
                'raw_json': json_str
            }
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            # JSON解析失败
            return {
                'device_id': device_id,
                'command_code': command_code,
                'command_name': self._get_command_name(command_code),
                'json_data': None,
                'raw_json': json_bytes.hex(),
                'error': str(e)
            }

    def feed_data(self, data: bytes) -> list[Dict[str, Any]]:
        """
        喂入数据到缓冲区并尝试解析完整帧

        由于TCP是流式传输，一次recv可能收到：
        - 不完整的帧
        - 一个完整的帧
        - 多个完整的帧
        - 一个完整的帧 + 下一个帧的一部分

        此方法处理这些情况，返回所有成功解析的完整帧。

        Args:
            data: 新接收的字节流

        Returns:
            解析出的完整帧列表（可能为空）

        Examples:
            >>> protocol = InkjetProtocol()
            >>> # 分两次接收一个完整帧
            >>> part1 = bytes.fromhex('1b0200157b224555')
            >>> result1 = protocol.feed_data(part1)
            >>> len(result1)
            0
            >>> part2 = bytes.fromhex('324c223a7b226e6f69736573223a317d7d')
            >>> result2 = protocol.feed_data(part2)
            >>> len(result2)
            1
            >>> result2[0]['json_data']
            {'EU2L': {'noises': 1}}
        """
        # 将新数据追加到缓冲区
        self._buffer.extend(data)

        frames = []

        while True:
            # 尝试查找帧头
            frame_start = self._buffer.find(self.FRAME_HEADER)

            if frame_start == -1:
                # 没有找到帧头，清空无效数据
                if len(self._buffer) >= 100:  # 防止缓冲区无限增长
                    self._buffer.clear()
                break

            # 丢弃帧头之前的垃圾数据
            if frame_start > 0:
                self._buffer = self._buffer[frame_start:]

            # 检查是否有足够的数据读取头部
            if len(self._buffer) < self.HEADER_TOTAL_LENGTH:
                break

            # 尝试找到JSON的结束位置
            # 策略：查找JSON的花括号配对
            json_start = self.HEADER_TOTAL_LENGTH
            frame_end = self._find_json_end(self._buffer, json_start)

            if frame_end == -1:
                # JSON不完整，等待更多数据
                # 但如果缓冲区太大，可能是格式错误
                if len(self._buffer) > 4096:  # 4KB限制
                    # 丢弃这个损坏的帧
                    self._buffer = self._buffer[self.HEADER_TOTAL_LENGTH:]
                    continue
                break

            # 提取完整帧
            frame_data = bytes(self._buffer[:frame_end])

            # 解码帧
            decoded = self.decode_frame(frame_data)
            if decoded:
                frames.append(decoded)

            # 从缓冲区移除已处理的帧
            self._buffer = self._buffer[frame_end:]

        return frames

    def _find_json_end(self, data: bytearray, start_pos: int) -> int:
        """
        查找JSON字符串的结束位置

        通过匹配花括号来判断JSON的完整性。

        Args:
            data: 数据缓冲区
            start_pos: JSON开始位置

        Returns:
            JSON结束位置（下一个字节的索引），如果未找到返回 -1
        """
        if start_pos >= len(data):
            return -1

        # 检查第一个字符是否是 '{'
        if data[start_pos] != ord('{'):
            # 不是JSON对象，可能是其他格式或空数据
            # 尝试查找换行符或其他分隔符
            # 对于此协议，我们假设必须是JSON对象
            return -1

        brace_count = 0
        in_string = False
        escape_next = False

        for i in range(start_pos, len(data)):
            char = chr(data[i])

            if escape_next:
                escape_next = False
                continue

            if char == '\\':
                escape_next = True
                continue

            if char == '"':
                in_string = not in_string
                continue

            if in_string:
                continue

            if char == '{':
                brace_count += 1
            elif char == '}':
                brace_count -= 1

                if brace_count == 0:
                    # 找到完整的JSON对象
                    return i + 1

        # JSON不完整
        return -1

    def _get_command_name(self, command_code: int) -> str:
        """
        获取指令名称

        Args:
            command_code: 指令码

        Returns:
            指令名称字符串，未知指令返回 "UNKNOWN"
        """
        try:
            return InkjetCommand(command_code).name
        except ValueError:
            return f"UNKNOWN_0x{command_code:02X}"

    def clear_buffer(self) -> None:
        """清空接收缓冲区"""
        self._buffer.clear()

    def get_buffer_size(self) -> int:
        """获取当前缓冲区大小"""
        return len(self._buffer)


# 便捷函数
def create_command(
    device_id: int,
    command_code: int,
    json_data: Dict[str, Any]
) -> bytes:
    """
    创建打印机命令（便捷函数）

    Args:
        device_id: 设备号 (0-255)
        command_code: 指令码
        json_data: JSON数据

    Returns:
        编码后的字节流

    Examples:
        >>> cmd = create_command(0, InkjetCommand.NOISES, {"EU2L": {"noises": 1}})
        >>> cmd.hex()
        '1b0200157b224555324c223a7b226e6f69736573223a317d7d'
    """
    protocol = InkjetProtocol()
    return protocol.encode_command(device_id, command_code, json_data)


def parse_command(data: bytes) -> Optional[Dict[str, Any]]:
    """
    解析打印机命令（便捷函数）

    Args:
        data: 字节流

    Returns:
        解析结果字典，失败返回None

    Examples:
        >>> raw = bytes.fromhex('1b0200157b224555324c223a7b226e6f69736573223a317d7d')
        >>> result = parse_command(raw)
        >>> result['device_id']
        0
        >>> result['json_data']
        {'EU2L': {'noises': 1}}
    """
    protocol = InkjetProtocol()
    return protocol.decode_frame(data)
