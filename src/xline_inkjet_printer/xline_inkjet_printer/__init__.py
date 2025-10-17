"""
xline_inkjet_printer - 喷墨打印机控制包

提供 ROS 2 喷墨打印机的完整控制解决方案。
"""

# 协议层
from .protocol import (
    InkjetCommand,
    InkjetProtocol,
    create_command,
    parse_command
)

# 命令模板
from .command_templates import (
    PrinterCommandTemplates,
    beep,
    start_print,
    stop_print,
    clean_nozzle,
    BEEP,
    START_PRINT,
    STOP_PRINT,
    CLEAN_NOZZLE
)

# 客户端工具
from .printer_client import (
    PrinterClient,
    MultiPrinterClient
)

# 墨盒查询
from .ink_level_query import (
    InkLevelQuery,
    query_ink_level_async,
    query_ink_level_sync
)

# 节点
from .async_inkjet_node import AsyncInkjetPrinterNode

# 版本信息
__version__ = '1.0.0'

__all__ = [
    # 协议层
    'InkjetCommand',
    'InkjetProtocol',
    'create_command',
    'parse_command',

    # 命令模板
    'PrinterCommandTemplates',
    'beep',
    'start_print',
    'stop_print',
    'clean_nozzle',
    'BEEP',
    'START_PRINT',
    'STOP_PRINT',
    'CLEAN_NOZZLE',

    # 客户端工具
    'PrinterClient',
    'MultiPrinterClient',

    # 墨盒查询
    'InkLevelQuery',
    'query_ink_level_async',
    'query_ink_level_sync',

    # 节点
    'AsyncInkjetPrinterNode',
]
