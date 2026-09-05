"""提供导航执行器和目标端口的统一实现入口。"""

# 重新导出路由执行器及其身份绑定终局闸门，供组合根装配使用。
from .router import BoundCompletionSink, RealExecutor, RoutedExecutor

# 声明执行层模块的稳定公开类型。
__all__ = (
    "BoundCompletionSink",
    "RoutedExecutor",
    "RealExecutor",
)
