"""出发剧本的窄门面，避免调用方依赖 Choreographer 内部展开细节。"""


class DepartureChoreographyFactory:
    """为出发分析器提供固定启动与左转替代剧本。"""

    def __init__(self, choreographer):
        self._choreographer = choreographer

    def start_departure(self):
        """生成 START 到 J_START 后固定右转并观察的剧本。"""
        return self._choreographer.start_departure()

    def start_left_turn(self):
        """生成撤回右转后左转并观察的剧本。"""
        return self._choreographer.start_departure_left_turn()

