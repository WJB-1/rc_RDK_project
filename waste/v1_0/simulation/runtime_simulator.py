"""导航运行时的确定性仿真外壳。

``NavigationRuntime`` 只负责导航决策，不会自己发送动作、产生里程计
或返回底盘执行结果。本文件模拟那个“外部世界”，因此也展示了将来
真实 ``main.py`` 需要承担的闭环职责：

    NavigationRuntime 产生 ActionCommand
        -> Simulator/真实底盘执行
        -> 产生 OdomUpdate、MapUpdateIntent、ActionFeedback
        -> NavigationRuntime.dispatch() 接收并更新下一步状态

仿真器不直接改 ``RuntimeMap`` 或动作队列；它只像真实设备那样报告
发生的事实。这使得仿真路径和未来真实机器人使用相同的公开接口。
"""

from dataclasses import dataclass, field

from ..contracts import ActionFeedback, MapUpdateIntent, OdomUpdate
from ..runtime import NavigationRuntime


@dataclass
class SimClock:
    """可手动推进的虚拟时间。

    不调用真实 ``time.sleep()``，所以同一份输入始终得到同样的结果，
    测试也不会因为电脑速度不同而不稳定。
    """

    now: float = 0.0

    def __call__(self) -> float:
        """让实例可以像函数一样作为 ``clock()`` 使用。"""
        return self.now

    def advance(self, seconds: float) -> None:
        """向前推进虚拟时间；负数不会使时间倒退。"""
        self.now += max(0.0, seconds)


@dataclass
class SimWorld:
    """仿真世界中尚未被机器人观察到的事实。

    ``pending_map_intents`` 相当于赛道中真实存在、但要到 ``observe``
    动作时才会被相机发现的障碍或涵洞；导航系统不能直接读取它。
    """
    pending_map_intents: list[MapUpdateIntent] = field(default_factory=list)
    emitted_intents: list[MapUpdateIntent] = field(default_factory=list)

    def observe(self) -> tuple[MapUpdateIntent, ...]:
        """返回本次观察可见的事实，并将其标记为已发出。

        这里返回的是事件数据包，而不是直接修改导航地图。这对应真实
        系统的“视觉模块检测到事实，再上报给导航系统”。
        """
        intents = tuple(self.pending_map_intents)
        self.pending_map_intents.clear()
        self.emitted_intents.extend(intents)
        return intents


class RuntimeSimulator:
    """替代真实 ``main.py + 底盘 + 传感器`` 的外层闭环。

    它不知道如何规划路线；路线仍由 ``NavigationRuntime`` 决定。它的
    职责是反复取出一条 ``ActionCommand``，模拟执行，并将执行结果用
    事件回灌给运行时。
    """

    def __init__(self, runtime: NavigationRuntime, world: SimWorld | None = None,
                 clock: SimClock | None = None):
        # 可以传入固定的世界和时钟，使测试场景完全可复现。
        self.clock = clock or SimClock()
        self.runtime = runtime
        self.world = world or SimWorld()
        # 把导航运行时使用的时钟替换成仿真时钟。
        self.runtime.clock = self.clock
        # 真实系统会由 main.py 调用 start()；仿真器承担这个启动责任。
        self.runtime.start()

    def step(self) -> bool:
        """完整模拟一条动作，并把产生的事件返回给导航运行时。

        返回 ``True`` 表示完成了一条动作；返回 ``False`` 表示当前既无
        动作、也无法产生新计划，外部循环可以停止。
        """
        if self.runtime.next_action() is None:
            # 动作队列为空时，外部执行循环应请求导航系统进行规划。
            self.runtime.plan()
        action = self.runtime.next_action()
        if action is None:
            return False

        # ActionCommand 使用不可变的参数元组；转换为字典便于读取距离。
        parameters = dict(action.parameters)
        if action.kind in {"drive", "drive_to_observation"}:
            # 行驶会改变位姿。仿真不改 pose，而是像底盘一样上报里程计。
            distance = float(parameters.get("distance_mm", 0.0))
            self.runtime.dispatch(OdomUpdate(dy_mm=distance, timestamp=self.clock.now))
            # 以 300 mm/s 的假定速度推进虚拟时间，至少消耗 0.01 秒。
            self.clock.advance(max(0.01, distance / 300.0))
        elif action.kind == "observe":
            # 只有“观察”动作才让世界中的待发现事实进入导航系统。
            for intent in self.world.observe():
                self.runtime.dispatch(intent)
        else:
            # 转弯、任务等动作暂未建立详细运动模型，只消耗极短虚拟时间。
            self.clock.advance(0.01)

        # 无论是哪种动作，最后都由“底盘”报告同一个 action_id 已成功完成。
        # 编排器据此将动作游标推进到下一条命令。
        self.runtime.dispatch(ActionFeedback(action.action_id, "succeeded", timestamp=self.clock.now))
        return True

    def publish_world_events(self) -> int:
        """不等待 ``observe`` 动作，主动把世界事件逐条上报给导航系统。

        这主要用于测试“地图突然更新，旧路线应当中断”的情景。
        """
        count = 0
        for intent in self.world.observe():
            self.runtime.dispatch(intent)
            count += 1
        return count

    def run_actions(self, limit: int = 100) -> int:
        """连续执行动作，直至无动作可执行或达到安全上限。"""
        completed = 0
        for _ in range(limit):
            if not self.step():
                break
            completed += 1
        return completed
