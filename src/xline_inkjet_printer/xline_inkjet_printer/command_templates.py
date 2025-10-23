"""
喷墨打印机常用命令模板

提供预定义的常用命令，简化用户操作。
基于 printer_cmd.txt 中列举的常用指令。
"""

from typing import Dict, Any, Tuple
from .protocol import InkjetCommand


class PrinterCommandTemplates:
    """
    打印机命令模板库

    提供常用指令的完整参数模板，用户无需记忆复杂的JSON格式。
    每个方法返回 (command_code, json_data) 元组。
    """

    @staticmethod
    def beep(times: int = 1) -> Tuple[int, Dict[str, Any]]:
        """
        蜂鸣指令

        Args:
            times: 蜂鸣次数，默认1次

        Returns:
            (command_code, json_data) 元组

        Examples:
            >>> cmd, data = PrinterCommandTemplates.beep()
            >>> cmd
            21  # 0x15
            >>> data
            {'EU2L': {'noises': 1}}
        """
        return (
            InkjetCommand.NOISES,
            {"EU2L": {"noises": times}}
        )

    @staticmethod
    def start_print() -> Tuple[int, Dict[str, Any]]:
        """
        开启打印

        Returns:
            (command_code, json_data) 元组

        Examples:
            >>> cmd, data = PrinterCommandTemplates.start_print()
            >>> data
            {'EU2L': {'setupEvent': 1}}
        """
        return (
            InkjetCommand.SETUP_EVENT,
            {"EU2L": {"setupEvent": 1}}
        )

    @staticmethod
    def stop_print() -> Tuple[int, Dict[str, Any]]:
        """
        关闭打印

        Returns:
            (command_code, json_data) 元组

        Examples:
            >>> cmd, data = PrinterCommandTemplates.stop_print()
            >>> data
            {'EU2L': {'setupEvent': 0}}
        """
        return (
            InkjetCommand.SETUP_EVENT,
            {"EU2L": {"setupEvent": 0}}
        )

    @staticmethod
    def clean_nozzle(intensity: int = 20, ink_box: int = 0) -> Tuple[int, Dict[str, Any]]:
        """
        清洗喷头

        Args:
            intensity: 清洗强度，默认20
            ink_box: 墨盒编号，默认0

        Returns:
            (command_code, json_data) 元组

        Examples:
            >>> cmd, data = PrinterCommandTemplates.clean_nozzle()
            >>> data
            {'InkBox_0': {'clean': 20}}

            >>> cmd, data = PrinterCommandTemplates.clean_nozzle(intensity=30, ink_box=1)
            >>> data
            {'InkBox_1': {'clean': 30}}
        """
        return (
            InkjetCommand.SETUP_EVENT,
            {f"InkBox_{ink_box}": {"clean": intensity}}
        )

    @staticmethod
    def test_print(
        text: str = "12345",
        file_name: str = "txt.msg",
        font_family: str = "Arial-MonoBold",
        pixel_size: int = 140,
        x: int = 476,
        y: int = -3
    ) -> Tuple[int, Dict[str, Any]]:
        """
        测试打印指令（完整版）

        发送完整的测试打印内容，包含文本和装饰图形，用于验证打印机功能。
        该指令复制自实际硬件测试指令，包含13个打印模块：
        - 1个文本模块：显示测试文本
        - 12个图片模块：在文本周围绘制矩形边框

        Args:
            text: 打印文本内容，默认"12345"
            file_name: 文件名，默认"txt.msg"
            font_family: 字体系列，默认"Arial-MonoBold"
            pixel_size: 像素大小，默认140
            x: X坐标位置，默认476
            y: Y坐标位置，默认-3

        Returns:
            (command_code, json_data) 元组

        Examples:
            >>> cmd, data = PrinterCommandTemplates.test_print()
            >>> cmd
            84  # 0x54
            >>> len(data['Mesg']['modules'])
            13  # 1个文本 + 12个图片
            >>> data['Mesg']['modules'][0]['text']
            '12345'

            >>> cmd, data = PrinterCommandTemplates.test_print(text="TEST")
            >>> data['Mesg']['modules'][0]['text']
            'TEST'
        """
        # 图片模块的Base64图像数据（来自实际硬件协议）
        img_data = "Zlib64:AAAAPXicYyhkYPlPJvgAAIp3OS4="

        return (
            InkjetCommand.TEST,
            {
                "Mesg": {
                    "fileName": file_name,
                    "modules": [
                        # 模块1: 文本模块 - 显示测试文本
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
                            "y": y
                        },
                        # 模块2-9: 水平图片模块 - 绘制水平线
                        {
                            "direc": 0,
                            "fileName": "矩形 1(1).bmp",
                            "height": 5,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 113,
                            "scale": 1,
                            "width": 113,
                            "x": 150,
                            "y": 10
                        },
                        {
                            "direc": 0,
                            "fileName": "矩形 1(1).bmp",
                            "height": 5,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 113,
                            "scale": 1,
                            "width": 113,
                            "x": 150,
                            "y": 25
                        },
                        {
                            "direc": 0,
                            "fileName": "矩形 1(1).bmp",
                            "height": 5,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 113,
                            "scale": 1,
                            "width": 113,
                            "x": 150,
                            "y": 40
                        },
                        {
                            "direc": 0,
                            "fileName": "矩形 1(1).bmp",
                            "height": 5,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 113,
                            "scale": 1,
                            "width": 113,
                            "x": 150,
                            "y": 58
                        },
                        {
                            "direc": 0,
                            "fileName": "矩形 1(1).bmp",
                            "height": 5,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 113,
                            "scale": 1,
                            "width": 113,
                            "x": 150,
                            "y": 78
                        },
                        {
                            "direc": 0,
                            "fileName": "矩形 1(1).bmp",
                            "height": 5,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 113,
                            "scale": 1,
                            "width": 113,
                            "x": 150,
                            "y": 96
                        },
                        {
                            "direc": 0,
                            "fileName": "矩形 1(1).bmp",
                            "height": 5,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 113,
                            "scale": 1,
                            "width": 113,
                            "x": 150,
                            "y": 117
                        },
                        {
                            "direc": 0,
                            "fileName": "矩形 1(1).bmp",
                            "height": 5,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 113,
                            "scale": 1,
                            "width": 113,
                            "x": 150,
                            "y": 137
                        },
                        # 模块10-13: 垂直图片模块 - 绘制垂直线
                        {
                            "direc": 90,
                            "fileName": "矩形 1(1).bmp",
                            "height": 140,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 140,
                            "scale": 1,
                            "width": 5,
                            "x": 316,
                            "y": 5
                        },
                        {
                            "direc": 90,
                            "fileName": "矩形 1(1).bmp",
                            "height": 140,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 140,
                            "scale": 1,
                            "width": 5,
                            "x": 348,
                            "y": 5
                        },
                        {
                            "direc": 90,
                            "fileName": "矩形 1(1).bmp",
                            "height": 140,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 140,
                            "scale": 1,
                            "width": 5,
                            "x": 380,
                            "y": 4
                        },
                        {
                            "direc": 90,
                            "fileName": "矩形 1(1).bmp",
                            "height": 140,
                            "img": img_data,
                            "inverse": False,
                            "mtype": 3,
                            "sHeight": 5,
                            "sWidth": 140,
                            "scale": 1,
                            "width": 5,
                            "x": 411,
                            "y": 5
                        }
                    ]
                }
            }
        )

    @staticmethod
    def custom_command(command_code: int, json_data: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """
        自定义命令

        Args:
            command_code: 指令码
            json_data: JSON数据

        Returns:
            (command_code, json_data) 元组
        """
        return (command_code, json_data)


# 便捷函数别名
def beep(times: int = 1) -> Tuple[int, Dict[str, Any]]:
    """蜂鸣 - 便捷函数"""
    return PrinterCommandTemplates.beep(times)


def start_print() -> Tuple[int, Dict[str, Any]]:
    """开启打印 - 便捷函数"""
    return PrinterCommandTemplates.start_print()


def stop_print() -> Tuple[int, Dict[str, Any]]:
    """关闭打印 - 便捷函数"""
    return PrinterCommandTemplates.stop_print()


def clean_nozzle(intensity: int = 20, ink_box: int = 0) -> Tuple[int, Dict[str, Any]]:
    """清洗喷头 - 便捷函数"""
    return PrinterCommandTemplates.clean_nozzle(intensity, ink_box)


def test_print(
    text: str = "12345",
    file_name: str = "txt.msg",
    font_family: str = "Arial-MonoBold",
    pixel_size: int = 140,
    x: int = 476,
    y: int = -3
) -> Tuple[int, Dict[str, Any]]:
    """测试打印 - 便捷函数"""
    return PrinterCommandTemplates.test_print(text, file_name, font_family, pixel_size, x, y)


# 预定义常量（快速访问）
BEEP = PrinterCommandTemplates.beep()
START_PRINT = PrinterCommandTemplates.start_print()
STOP_PRINT = PrinterCommandTemplates.stop_print()
CLEAN_NOZZLE = PrinterCommandTemplates.clean_nozzle()
TEST_PRINT = PrinterCommandTemplates.test_print()
