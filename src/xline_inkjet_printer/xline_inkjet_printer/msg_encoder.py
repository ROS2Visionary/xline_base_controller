"""
消息编码器模块

提供所有打印机指令的 JSON 构造和编码功能。
所有指令的构造和编码统一在此模块完成。

设计原则：
1. build_*_json() 函数：返回 Dict[str, Any]，供上层灵活使用
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

def build_beep_json(times: int = 1) -> Dict[str, Any]:
    """
    构造蜂鸣指令的 JSON 数据。

    Args:
        times: 蜂鸣次数

    Returns:
        JSON 数据字典
    """
    return {"EU2L": {"noises": times}}


def encode_beep(times: int = 1) -> bytes:
    """
    蜂鸣指令：返回 JSON 字节流（不带前缀）。

    Args:
        times: 蜂鸣次数

    Returns:
        编码后的字节流
    """
    return encode_dict_to_bytes(build_beep_json(times))


# =============================================================================
# 控制指令 - 开始/停止打印
# =============================================================================

def build_start_print_json() -> Dict[str, Any]:
    """
    构造开启打印指令的 JSON 数据。

    Returns:
        JSON 数据字典
    """
    return {"EU2L": {"setupEvent": 1}}


def build_stop_print_json() -> Dict[str, Any]:
    """
    构造关闭打印指令的 JSON 数据。

    Returns:
        JSON 数据字典
    """
    return {"EU2L": {"setupEvent": 0}}


def encode_start_print() -> bytes:
    """
    开启打印指令：返回 JSON 字节流（不带前缀）。

    Returns:
        编码后的字节流
    """
    return encode_dict_to_bytes(build_start_print_json())


def encode_stop_print() -> bytes:
    """
    关闭打印指令：返回 JSON 字节流（不带前缀）。

    Returns:
        编码后的字节流
    """
    return encode_dict_to_bytes(build_stop_print_json())


# =============================================================================
# 维护指令 - 清洗喷头
# =============================================================================

def build_clean_nozzle_json(intensity: int = 20, ink_box: int = 0) -> Dict[str, Any]:
    """
    构造清洗喷头指令的 JSON 数据。

    Args:
        intensity: 清洗强度（默认20）
        ink_box: 墨盒编号（默认0）

    Returns:
        JSON 数据字典
    """
    return {f"InkBox_{ink_box}": {"clean": intensity}}


def encode_clean_nozzle(intensity: int = 20, ink_box: int = 0) -> bytes:
    """
    清洗喷头指令：返回 JSON 字节流（不带前缀）。

    Args:
        intensity: 清洗强度
        ink_box: 墨盒编号

    Returns:
        编码后的字节流
    """
    return encode_dict_to_bytes(build_clean_nozzle_json(intensity, ink_box))


# =============================================================================
# 模式设置 - 打印模式
# =============================================================================

def build_print_mode_json(
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
        >>> build_print_mode_json(interval=75, is_full_end=0, mode=1)
        {'PrintMode': {'interval': 75, 'isFullEnd': 0, 'mode': 1}}
    """
    return {
        "PrintMode": {
            "interval": interval,
            "isFullEnd": is_full_end,
            "mode": mode
        }
    }


def encode_print_mode(
    interval: int = 75,
    is_full_end: int = 0,
    mode: int = 1
) -> bytes:
    """
    打印模式设置指令：返回 JSON 字节流（不带前缀）。

    Args:
        interval: 打印间隔（毫秒）
        is_full_end: 是否整段结束标志
        mode: 打印模式

    Returns:
        编码后的字节流
    """
    return encode_dict_to_bytes(build_print_mode_json(interval, is_full_end, mode))


# 旧版兼容函数（返回 hex 字符串，建议使用新版）

def encode_printmode_count(interval: float = 1809) -> str:
    """
    计数模式（打字）- 旧版兼容。

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
    return encode_dict_to_hex(data)


def encode_printmode_line(interval: float = 12, is_full_end: int = 0) -> str:
    """
    连续模式（画线）- 旧版兼容。

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
    return encode_dict_to_hex(data)


# =============================================================================
# 消息指令 - 文本消息
# =============================================================================

def build_text_message_json(
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


def encode_text_message(
    text: str,
    file_name: str = "txt.msg",
    font_family: str = "Arial-MonoBold",
    pixel_size: int = 310,
    x: int = 0,
    y: int = 0
) -> bytes:
    """
    文本消息指令：返回 JSON 字节流（不带前缀）。

    Args:
        text: 要打印的文本
        file_name: 消息文件名
        font_family: 字体
        pixel_size: 像素大小
        x: X 坐标
        y: Y 坐标

    Returns:
        编码后的字节流
    """
    return encode_dict_to_bytes(
        build_text_message_json(text, file_name, font_family, pixel_size, x, y)
    )


# 旧版兼容函数

def encode_text_to_msg_hex(text: str, with_prefix: bool = False) -> str:
    """
    文本协议 - 旧版兼容。

    Args:
        text: 要打印的文本
        with_prefix: 已废弃参数，不再使用

    Returns:
        JSON 部分的 hex 字符串

    Note:
        建议使用 encode_text_message() 的 bytes 版本
    """
    return encode_dict_to_hex(build_text_message_json(text))


# =============================================================================
# 消息指令 - 单条线段
# =============================================================================

# 通用图片数据（用于线段/矩形）
_IMG_DATA = "Zlib64:AAAAPXicYyhkYPlPJvgAAIp3OS4="


def build_single_line_json(
    height: int = 5,
    width: int = 150,
    x: int = 0,
    y: int = 75,
    file_name: str = "line_1.msg"
) -> Dict[str, Any]:
    """
    构造单条线段消息的 JSON 数据。

    Args:
        height: 线段高度（像素），默认5
        width: 线段宽度（像素），默认150
        x: X 坐标，默认0
        y: Y 坐标，默认75
        file_name: 消息文件名

    Returns:
        JSON 数据字典

    Examples:
        >>> build_single_line_json(height=5, width=150, x=0, y=75)
        {'Mesg': {'fileName': 'line_1.msg', 'modules': [...]}}
    """
    return {
        "Mesg": {
            "fileName": file_name,
            "modules": [
                {
                    "direc": 0,
                    "fileName": " 1(1).bmp",
                    "height": height,
                    "img": _IMG_DATA,
                    "inverse": False,
                    "mtype": 3,
                    "sHeight": height,
                    "sWidth": width,
                    "scale": 1,
                    "width": width,
                    "x": x,
                    "y": y,
                }
            ],
        }
    }


def encode_single_line(
    height: int = 5,
    width: int = 150,
    x: int = 0,
    y: int = 75,
    file_name: str = "line_1.msg"
) -> bytes:
    """
    单条线段消息指令：返回 JSON 字节流（不带前缀）。

    Args:
        height: 线段高度（像素）
        width: 线段宽度（像素）
        x: X 坐标
        y: Y 坐标
        file_name: 消息文件名

    Returns:
        编码后的字节流
    """
    return encode_dict_to_bytes(
        build_single_line_json(height, width, x, y, file_name)
    )


# =============================================================================
# 消息指令 - 测试打印
# =============================================================================

def build_test_print_modules(
    text: str = "12345",
    font_family: str = "Arial-MonoBold",
    pixel_size: int = 140,
    x: int = 476,
    y: int = -3,
) -> List[Dict[str, Any]]:
    """
    构造测试打印的 modules 列表（1 个文本 + 12 个图片）。

    Args:
        text: 测试文本
        font_family: 字体
        pixel_size: 像素大小
        x: 文本 X 坐标
        y: 文本 Y 坐标

    Returns:
        模块列表（13个模块）
    """
    modules: List[Dict[str, Any]] = []

    # 模块1: 文本模块
    modules.append(
        {
            "direc": 0,
            "family": font_family,
            "height": 159,
            "letterSpace": 0,
            "mtype": 0,
            "pixelSize": pixel_size,
            "text": text,
            "width": 420,
            "x": x,
            "y": y,
        }
    )

    # 模块2-9: 水平矩形
    y_values = [10, 25, 40, 58, 78, 96, 117, 137]
    for y_val in y_values:
        modules.append(
            {
                "direc": 0,
                "fileName": "矩形 1(1).bmp",
                "height": 5,
                "img": _IMG_DATA,
                "inverse": False,
                "mtype": 3,
                "sHeight": 5,
                "sWidth": 113,
                "scale": 1,
                "width": 113,
                "x": 150,
                "y": y_val,
            }
        )

    # 模块10-13: 垂直矩形
    x_values = [316, 348, 380, 411]
    for x_val in x_values:
        modules.append(
            {
                "direc": 90,
                "fileName": "矩形 1(1).bmp",
                "height": 140,
                "img": _IMG_DATA,
                "inverse": False,
                "mtype": 3,
                "sHeight": 5,
                "sWidth": 140,
                "scale": 1,
                "width": 5,
                "x": x_val,
                "y": 5 if x_val != 380 else 4,
            }
        )

    return modules


def build_test_print_json(
    text: str = "12345",
    file_name: str = "txt.msg",
    font_family: str = "Arial-MonoBold",
    pixel_size: int = 140,
    x: int = 476,
    y: int = -3,
) -> Dict[str, Any]:
    """
    构造测试打印指令的 JSON 数据。

    Args:
        text: 测试文本
        file_name: 消息文件名
        font_family: 字体
        pixel_size: 像素大小
        x: 文本 X 坐标
        y: 文本 Y 坐标

    Returns:
        JSON 数据字典
    """
    return {
        "Mesg": {
            "fileName": file_name,
            "modules": build_test_print_modules(
                text=text,
                font_family=font_family,
                pixel_size=pixel_size,
                x=x,
                y=y,
            ),
        }
    }


def encode_test_print(
    text: str = "12345",
    file_name: str = "txt.msg",
    font_family: str = "Arial-MonoBold",
    pixel_size: int = 140,
    x: int = 476,
    y: int = -3,
) -> bytes:
    """
    测试打印指令：返回 JSON 字节流（不带前缀）。

    Args:
        text: 测试文本
        file_name: 消息文件名
        font_family: 字体
        pixel_size: 像素大小
        x: 文本 X 坐标
        y: 文本 Y 坐标

    Returns:
        编码后的字节流
    """
    return encode_dict_to_bytes(
        build_test_print_json(
            text=text,
            file_name=file_name,
            font_family=font_family,
            pixel_size=pixel_size,
            x=x,
            y=y,
        )
    )


# =============================================================================
# 导出列表
# =============================================================================

__all__ = [
    # 通用编码
    'encode_dict_to_bytes',
    'encode_dict_to_hex',

    # 蜂鸣
    'build_beep_json',
    'encode_beep',

    # 开始/停止打印
    'build_start_print_json',
    'build_stop_print_json',
    'encode_start_print',
    'encode_stop_print',

    # 清洗喷头
    'build_clean_nozzle_json',
    'encode_clean_nozzle',

    # 打印模式
    'build_print_mode_json',
    'encode_print_mode',
    'encode_printmode_count',  # 旧版兼容
    'encode_printmode_line',   # 旧版兼容

    # 文本消息
    'build_text_message_json',
    'encode_text_message',
    'encode_text_to_msg_hex',  # 旧版兼容

    # 单条线段
    'build_single_line_json',
    'encode_single_line',

    # 测试打印
    'build_test_print_modules',
    'build_test_print_json',
    'encode_test_print',
]


# =============================================================================
# 简单自测
# =============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("msg_encoder 自测")
    print("=" * 60)

    # 蜂鸣
    print("\n[蜂鸣]")
    print(f"JSON: {build_beep_json(3)}")
    print(f"Bytes: {encode_beep(3).hex()}")

    # 开始打印
    print("\n[开始打印]")
    print(f"JSON: {build_start_print_json()}")
    print(f"Bytes: {encode_start_print().hex()}")

    # 停止打印
    print("\n[停止打印]")
    print(f"JSON: {build_stop_print_json()}")
    print(f"Bytes: {encode_stop_print().hex()}")

    # 清洗喷头
    print("\n[清洗喷头]")
    print(f"JSON: {build_clean_nozzle_json(20, 0)}")
    print(f"Bytes: {encode_clean_nozzle(20, 0).hex()}")

    # 打印模式
    print("\n[打印模式]")
    print(f"JSON: {build_print_mode_json(75, 0, 1)}")
    print(f"Bytes: {encode_print_mode(75, 0, 1).hex()}")

    # 单条线段
    print("\n[单条线段]")
    print(f"JSON: {build_single_line_json(5, 150, 0, 75)}")
    print(f"Bytes长度: {len(encode_single_line(5, 150, 0, 75))}")

    # 测试打印
    print("\n[测试打印]")
    print(f"模块数量: {len(build_test_print_modules())}")
    print(f"Bytes长度: {len(encode_test_print())}")

    print("\n" + "=" * 60)
    print("自测完成")
    print("=" * 60)
