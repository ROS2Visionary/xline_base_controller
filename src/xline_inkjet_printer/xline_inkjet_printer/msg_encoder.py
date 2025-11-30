"""
消息编码器模块

提供所有打印机指令的 JSON 构造和编码功能。
所有指令的构造和编码统一在此模块完成。

设计原则：
1. encode_*_json() 函数：返回 Dict[str, Any]，供上层灵活使用
2. encode_*() 函数：返回 bytes，供 TCP 层直接发送
3. 协议前缀（1B 02 + 长度）由 AsyncTcpClient 在发送时统一添加

指令分类：
- 控制指令：蜂鸣、开始打印、停止打印
- 维护指令：清洗喷头
- 模式设置：打印模式（计数/连续）
- 消息指令：文本消息、测试打印、单条线段
"""

import json
from typing import Dict, Any, List


# =============================================================================
# 通用编码入口
# =============================================================================

def encode_dict_to_bytes(data: Dict[str, Any]) -> bytes:
    """
    通用编码入口：将 JSON 字典编码为 UTF-8 字节数组（不带任何协议前缀）。

    前缀（1B 02 + 长度）统一由 AsyncTcpClient 在发送到 TCP 时添加。

    Args:
        data: JSON 数据字典

    Returns:
        UTF-8 编码的字节流
    """
    json_str = json.dumps(data, ensure_ascii=False, separators=(',', ':'))
    return json_str.encode('utf-8')


def encode_dict_to_hex(data: Dict[str, Any]) -> str:
    """
    辅助函数：将 JSON 字典编码为纯 JSON 的 hex 字符串（不带前缀）。
    主要用于调试或离线查看 JSON 内容。

    Args:
        data: JSON 数据字典

    Returns:
        十六进制字符串
    """
    return encode_dict_to_bytes(data).hex()


# =============================================================================
# 控制指令 - 蜂鸣
# =============================================================================

def encode_beep_json(times: int = 1) -> Dict[str, Any]:
    """
    构造蜂鸣指令的 JSON 数据。

    Args:
        times: 蜂鸣次数

    Returns:
        JSON 数据字典
    """
    return {"EU2L": {"noises": times}}




# =============================================================================
# 控制指令 - 开始/停止打印
# =============================================================================

def encode_start_print_json() -> Dict[str, Any]:
    """
    构造开启打印指令的 JSON 数据。

    Returns:
        JSON 数据字典
    """
    return {"EU2L": {"setupEvent": 1}}


def encode_stop_print_json() -> Dict[str, Any]:
    """
    构造关闭打印指令的 JSON 数据。

    Returns:
        JSON 数据字典
    """
    return {"EU2L": {"setupEvent": 0}}



# =============================================================================
# 维护指令 - 清洗喷头
# =============================================================================

def encode_clean_nozzle_json(intensity: int = 20, ink_box: int = 0) -> Dict[str, Any]:
    """
    构造清洗喷头指令的 JSON 数据。

    Args:
        intensity: 清洗强度（默认20）
        ink_box: 墨盒编号（默认0）

    Returns:
        JSON 数据字典
    """
    return {f"InkBox_{ink_box}": {"clean": intensity}}



# =============================================================================
# 模式设置 - 打印模式
# =============================================================================

def encode_print_mode_json(
    interval: int = 75,
    is_full_end: int = 0,
    mode: int = 1
) -> Dict[str, Any]:
    """
    构造打印模式设置指令的 JSON 数据。

    Args:
        interval: 打印间隔（毫秒），默认75ms
        is_full_end: 是否整段结束标志（协议字段 isFullEnd），默认0
        mode: 打印模式（1=连续模式/画线，2=计数模式/打字），默认1

    Returns:
        JSON 数据字典

    Examples:
        >>> encode_print_mode_json(interval=75, is_full_end=0, mode=1)
        {'PrintMode': {'interval': 75, 'isFullEnd': 0, 'mode': 1}}
    """
    return {
        "PrintMode": {
            "interval": interval,
            "isFullEnd": is_full_end,
            "mode": mode
        }
    }



def encode_printmode_text_json(interval: float = 1809) -> str:
    """
    计数模式（打字）

    对应 JSON:
    {"PrintMode":{"couCount":1,"interval":<interval>,"mode":2}}

    Args:
        interval: 打印间隔

    Returns:
        JSON 部分的 hex 字符串（不带 1B 02 前缀）

    Note:
        建议使用 encode_print_mode() 的 bytes 版本
    """
    data = {
        "PrintMode": {
            "couCount": 1,
            "interval": interval,
            "mode": 2
        }
    }
    return data


def encode_printmode_line_json(interval: float = 12, is_full_end: int = 0) -> str:
    """
    连续模式（画线）

    对应 JSON:
    {"PrintMode":{"interval":<interval>,"isFullEnd":<is_full_end>,"mode":1}}

    Args:
        interval: 打印间隔
        is_full_end: 是否整段结束标志

    Returns:
        JSON 部分的 hex 字符串（不带 1B 02 前缀）

    Note:
        建议使用 encode_print_mode() 的 bytes 版本
    """
    data = {
        "PrintMode": {
            "interval": interval,
            "isFullEnd": is_full_end,
            "mode": 1
        }
    }
    return data


# =============================================================================
# 消息指令 - 文本消息
# =============================================================================

def encode_text_message_json(
    text: str,
    file_name: str = "txt.msg",
    font_family: str = "Arial-MonoBold",
    pixel_size: int = 310,
    x: int = 0,
    y: int = 0
) -> Dict[str, Any]:
    """
    构造文本消息的 JSON 数据。

    Args:
        text: 要打印的文本
        file_name: 消息文件名
        font_family: 字体
        pixel_size: 像素大小
        x: X 坐标
        y: Y 坐标

    Returns:
        JSON 数据字典
    """
    width = len(text) * pixel_size

    return {
        "Mesg": {
            "fileName": file_name,
            "modules": [{
                "direc": 0,
                "family": font_family,
                "height": 351,
                "letterSpace": 0,
                "mtype": 0,
                "pixelSize": pixel_size,
                "text": text,
                "width": width,
                "x": x,
                "y": y
            }]
        }
    }





# =============================================================================
# 消息指令 - 单条线段
# =============================================================================





# =============================================================================
# 导出列表
# =============================================================================

__all__ = [
    # 通用编码
    'encode_dict_to_bytes',

    # 蜂鸣
    'encode_beep_json',

    # 开始/停止打印
    'encode_start_print_json',
    'encode_stop_print_json',

    # 清洗喷头
    'encode_clean_nozzle_json',

    # 打印模式
    'encode_print_mode_json',

    'encode_printmode_text_json',

    'encode_text_message_json',

    'encode_printmode_line_json',
]

