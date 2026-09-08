"""定义 Web 仿真控制命令及其安全分发边界。"""

from dataclasses import dataclass
from enum import Enum
from typing import Any, Mapping


class SimulationCommand(Enum):
    """允许面板提交的仿真生命周期命令。"""
    START = "START"
    PAUSE = "PAUSE"
    RESUME = "RESUME"
    STEP = "STEP"
    STOP = "STOP"
    RESET = "RESET"


@dataclass(frozen=True)
class CommandResult:
    """命令分发结果，不暴露 Python 异常栈。"""
    ok: bool
    command: str
    error: str = ""
    value: Any = None

    def to_dict(self) -> dict:
        """转换为 HTTP 响应字典。"""
        result = {"ok": self.ok, "command": self.command}
        if self.error:
            result["error"] = self.error
        if self.value is not None:
            result["value"] = self.value
        return result


class CommandDispatcher:
    """将白名单命令映射到 Runner 公共方法。"""
    def __init__(self, runner: Any) -> None:
        if runner is None:
            raise TypeError("runner 不能为空")
        self._runner = runner

    def dispatch(self, command: str, payload: Mapping[str, Any] = None) -> CommandResult:
        """校验并执行一个控制命令；未知命令不会触碰 Runner。"""
        name = str(command or "").upper()
        try:
            selected = SimulationCommand(name)
        except ValueError:
            return CommandResult(False, name, "不支持的仿真命令")
        payload = dict(payload or {})
        try:
            if selected is SimulationCommand.RESET:
                seed = payload.get("seed", 0)
                if isinstance(seed, bool) or not isinstance(seed, int):
                    return CommandResult(False, name, "seed 必须是整数")
                value = self._runner.reset(seed)
            else:
                value = getattr(self._runner, selected.value.lower())()
            return CommandResult(True, name, value=value)
        except Exception as exc:
            return CommandResult(False, name, str(exc))

