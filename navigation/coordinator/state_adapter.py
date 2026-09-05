"""旧版分离状态依赖到统一导航状态端口的迁移适配器。"""

from typing import Optional

from navigation.domain import AbsoluteMapUpdate, RobotState


class LegacyNavigationStateAdapter:
    """兼容旧装配方式，供新 Coordinator 逐步迁移使用。"""

    def __init__(self, state_store=None, runtime_map=None) -> None:
        """保存旧版机器人状态仓和运行时地图依赖。"""

        self._state_store = state_store
        self._runtime_map = runtime_map

    def robot_state(self) -> Optional[RobotState]:
        """读取旧状态仓中的机器人快照。"""

        if self._state_store is None:
            return None
        return self._state_store.robot_state()

    def runtime_map_snapshot(self):
        """读取旧运行时地图的不可变快照。"""

        if self._runtime_map is None:
            raise RuntimeError("未装配运行时地图")
        return self._runtime_map.snapshot()

    def apply_map_update(self, update: AbsoluteMapUpdate) -> bool:
        """把地图事实转交给旧运行时地图。"""

        if self._runtime_map is None:
            raise RuntimeError("未装配运行时地图")
        return self._runtime_map.apply(update)

    def replace_robot_state(self, state: RobotState) -> None:
        """把新机器人状态转交给旧状态仓。"""

        if self._state_store is None:
            raise RuntimeError("未装配机器人状态仓")
        self._state_store.replace_robot_state(state)
