"""协调器分析器的公共决策类型和基础能力。"""

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from navigation.contracts import ExecutionInterrupt


class AnalyzerDecisionKind(Enum):
    """分析器交给 Coordinator 主循环的控制结果。"""

    # 已经向执行器提交了一条异步动作，主循环应暂停等待中断。
    DISPATCHED = "dispatched"
    # 当前没有可立即推进的业务，等待外部事件或后续回调。
    WAITING = "waiting"
    # 分析器已经完成一次主状态转移，主循环应重新选择分析器。
    TRANSITIONED = "transitioned"
    # 导航已经进入异常或任务完成等终局，不再继续派发。
    TERMINAL = "terminal"


@dataclass(frozen=True)
class AnalyzerDecision:
    """分析器单次运行的结果，不携带裸 Action。"""

    # 本次分析要求 Coordinator 主循环采取的控制动作。
    kind: AnalyzerDecisionKind
    # 供日志和调试面板显示的简短原因。
    reason: str = ""


class BaseAnalyzer:
    """四个主状态分析器共享的最小基础类。

    分析器只负责业务判断；动作必须先交给编排器生成剧本，
    再由 Coordinator 的统一执行入口提交给执行器。
    """

    def __init__(self, coordinator) -> None:
        """保存 Coordinator 服务引用，不复制任何运行上下文。"""

        self._coordinator = coordinator

    def run(self) -> AnalyzerDecision:
        """推进当前主状态一次；具体状态由子类实现。"""

        raise NotImplementedError

    def analyze_interrupt(self, interrupt: "ExecutionInterrupt") -> AnalyzerDecision:
        """分析执行器终局，并把通用身份校验交给 Coordinator。

        具体分析器可覆盖本方法，在公共投影完成后决定继续派发、
        重新规划或转换主状态；它不会直接创建或提交 Action。
        """

        handled = self._coordinator._handle_execution_interrupt_core(interrupt)
        if not handled:
            return AnalyzerDecision(
                AnalyzerDecisionKind.WAITING,
                "忽略未匹配的执行中断",
            )
        return AnalyzerDecision(
            AnalyzerDecisionKind.WAITING,
            "执行中断已完成公共投影，等待具体分析器继续判断",
        )
