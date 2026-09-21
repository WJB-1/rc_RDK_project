# -*- coding: utf-8 -*-
"""线程局部阶段计时器。

用法：
    from perception.algorithms.core.timing import reset_frame, block, get_frame_timings

    reset_frame()
    with block("edge.resize"):
        ...
    with block("edge.bpu_forward"):
        ...
    for name, ms in get_frame_timings():
        print(name, ms)

设计目标：
- 零分配：Block 用 __slots__
- 线程安全：threading.local
- 只负责收集，不做持久化（写入交给调用方）
"""
import threading
import time

_tls = threading.local()


def reset_frame():
    """每帧开始时调用，清空当前线程的计时列表。"""
    _tls.timings = []


def record(name, elapsed_ms):
    """手动记录一个阶段的耗时（毫秒）。"""
    timings = getattr(_tls, "timings", None)
    if timings is not None:
        timings.append((name, float(elapsed_ms)))


def get_frame_timings():
    """取当前线程本帧收集到的 [(name, ms), ...]。"""
    return list(getattr(_tls, "timings", []) or [])


class Block:
    __slots__ = ("name", "t0")

    def __init__(self, name):
        self.name = name
        self.t0 = 0.0

    def __enter__(self):
        self.t0 = time.perf_counter()
        return self

    def __exit__(self, *exc):
        record(self.name, (time.perf_counter() - self.t0) * 1000.0)


def block(name):
    """语法糖：with block('xxx'): ..."""
    return Block(name)
