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


# 预定义常量（快速访问）
BEEP = PrinterCommandTemplates.beep()
START_PRINT = PrinterCommandTemplates.start_print()
STOP_PRINT = PrinterCommandTemplates.stop_print()
CLEAN_NOZZLE = PrinterCommandTemplates.clean_nozzle()
