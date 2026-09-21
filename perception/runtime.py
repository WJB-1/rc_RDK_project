"""Perception 独占的相机取帧边界。"""


class CameraRuntime:
    """从已有相机源取得前视帧，不暴露底层相机对象。"""

    def __init__(self, frame_source, clock_ns):
        self._frame_source = frame_source
        self._clock_ns = clock_ns
        self._next_frame_id = 0

    def next_front_frame(self):
        self._next_frame_id += 1
        frames = self._frame_source.get_frames()
        return self._next_frame_id, self._clock_ns(), frames.get("front")
