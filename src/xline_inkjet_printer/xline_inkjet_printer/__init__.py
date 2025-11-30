"""
xline_inkjet_printer - 喷墨打印机控制包

提供 ROS 2 喷墨打印机的完整控制解决方案。
"""


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
    # 墨盒查询
    'InkLevelQuery',
    'query_ink_level_async',
    'query_ink_level_sync',

    # 节点
    'AsyncInkjetPrinterNode',
]
