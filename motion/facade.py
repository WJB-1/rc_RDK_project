"""Motion 的真机装配门面，连接导航执行器、视觉回正和底盘传输。"""

from .port import MotionPort


class MotionFacade:
    """作为 Motion 系统的唯一装配入口。"""

    def __init__(self, transport, correction_source, clock=None):
        self._correction_source = correction_source
        self._active_session_id = None
        self._port = MotionPort(transport, clock=clock, vision_port=self)

    @property
    def port(self):
        """返回注入 Navigation `RealExecutor` 的 Motion 目标端口。"""

        return self._port

    def start(self):
        """打开 Motion 的唯一 UART 传输并发起握手。"""

        self._port.start()

    def create_real_executor(self, other_ports=()):
        """创建真机 Navigation 执行器，并固定注入本 Motion 目标端口。"""

        from navigation.execution import RealExecutor

        return RealExecutor((self._port, *tuple(other_ports)))

    def stop(self):
        """停止视觉会话和 UART 传输。"""

        if self._active_session_id is not None:
            self.stop_correction(self._active_session_id)
        self._port.stop()

    def start_correction(self, session_id):
        """在 STM32 确认回正动作后启动同身份的视觉会话。"""

        if self._active_session_id is not None:
            raise RuntimeError("a correction session is already active")
        self._active_session_id = session_id
        try:
            self._correction_source.start(session_id, self._on_correction)
        except Exception:
            self._active_session_id = None
            raise

    def stop_correction(self, session_id):
        """只停止仍属于活动回正请求的视觉会话。"""

        if self._active_session_id != session_id:
            return
        self._active_session_id = None
        self._correction_source.stop(session_id)

    def _on_correction(self, session_id, offset_mm, target_yaw_deg):
        """将视觉的最新回正测量原样转发给当前 STM32 回正动作。"""

        if session_id != self._active_session_id:
            return
        if offset_mm is None or target_yaw_deg is None:
            self._port.send_vision_processing(session_id)
            return
        self._port.send_vision_correction(session_id, offset_mm, target_yaw_deg)
