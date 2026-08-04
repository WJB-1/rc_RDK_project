# -*- coding: utf-8 -*-

"""
BPU 语义分割引擎封装

基于 BiSeNet/PTQ/rdk_test.py 的 BPU 推理逻辑，封装为可在主工程中复用的类。
负责：
- 加载 .bin 模型 (pyeasy_dnn)
- BGR -> NV12 预处理
- BPU 前向推理
- 后处理：argmax -> resize 回原图尺寸 -> 二值化掩码
"""

import cv2
import numpy as np


def _bgr2nv12(image: np.ndarray) -> np.ndarray:
    """将 OpenCV 的 BGR 图片转换为 NV12 格式"""
    height, width = image.shape[:2]
    area = height * width
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = yuv420p[:area]
    uv_planar = yuv420p[area:].reshape((2, area // 4))
    uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))

    nv12 = np.zeros_like(yuv420p)
    nv12[:area] = y
    nv12[area:] = uv_packed
    return nv12


class SegmentationEngine:
    """
    BPU 语义分割推理引擎
    """

    def __init__(self, model_path: str, input_size: int = 512, target_class: int = 1):
        """
        :param model_path: BPU .bin 模型路径
        :param input_size: 模型输入正方形边长（通常为 512）
        :param target_class: 目标类别索引（道路=1）
        """
        self.input_size = input_size
        self.target_class = target_class

        try:
            from hobot_dnn import pyeasy_dnn as dnn
        except ImportError as e:
            raise ImportError(
                "缺少 hobot_dnn 依赖，请确认在 RDK X5 环境上运行。"
            ) from e

        models = dnn.load(model_path)
        self.model = models[0]

    def inference(self, frame: np.ndarray, debug_timing: bool = False) -> np.ndarray:
        """
        对单帧 BGR 图像进行 BPU 推理，返回与原图同尺寸的二值化掩码。

        :param frame: BGR 格式 numpy 数组 (H, W, 3)
        :param debug_timing: 是否打印各阶段耗时
        :return: uint8 掩码 (H, W)，255 表示目标赛道，0 表示背景
        """
        import time as _time
        _t = {}

        if frame is None or frame.size == 0:
            raise ValueError("输入帧无效")

        h_orig, w_orig = frame.shape[:2]
        _t0 = _time.time()

        # 1) Resize 到模型输入尺寸
        img_resized = cv2.resize(frame, (self.input_size, self.input_size))
        _t["resize_down"] = (_time.time() - _t0) * 1000

        # 2) BGR -> NV12
        _t1 = _time.time()
        nv12_data = _bgr2nv12(img_resized)
        _t["bgr2nv12"] = (_time.time() - _t1) * 1000

        # 3) BPU 前向推理
        _t2 = _time.time()
        outputs = self.model.forward([nv12_data])
        preds = outputs[0].buffer
        _t["bpu_forward"] = (_time.time() - _t2) * 1000

        # 4) argmax
        _t3 = _time.time()
        mask_512 = np.argmax(preds[0], axis=0).astype(np.uint8)
        _t["argmax"] = (_time.time() - _t3) * 1000

        # 5) resize 回原始尺寸
        _t4 = _time.time()
        mask_orig = cv2.resize(
            mask_512, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST
        )
        _t["resize_up"] = (_time.time() - _t4) * 1000

        # 6) 二值化
        _t5 = _time.time()
        binary_mask = (mask_orig == self.target_class).astype(np.uint8) * 255
        _t["binary"] = (_time.time() - _t5) * 1000

        _t["total"] = (_time.time() - _t0) * 1000

        if debug_timing:
            print(f"[BPU timing] total={_t['total']:.1f}ms | "
                  f"resize_down={_t['resize_down']:.1f} bgr2nv12={_t['bgr2nv12']:.1f} "
                  f"bpu_forward={_t['bpu_forward']:.1f} argmax={_t['argmax']:.1f} "
                  f"resize_up={_t['resize_up']:.1f} binary={_t['binary']:.1f}")

        return binary_mask
