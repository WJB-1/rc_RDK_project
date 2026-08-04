# -*- coding: utf-8 -*-

"""
BPU 目标检测引擎封装 — YOLOv8 路口检测

基于 YOLOv8n + RDK X5 BPU (bayes-e) 编译的 .bin 模型。
负责：
- 加载 .bin 模型 (pyeasy_dnn)
- BGR -> NV12 预处理
- BPU 前向推理
- 后处理：解码 YOLOv8 输出 → NMS → 检测框列表
"""

import cv2
import numpy as np

_DEFAULT_MODEL = "models/yolov8_detection_x5.bin"

# YOLOv8n 输出: [1, 14, 8400] — 4 bbox + 10 class scores
_NUM_CLASSES = 10
_NUM_OUTPUTS = 4 + _NUM_CLASSES  # 14
_NUM_ANCHORS = 8400
_INPUT_SIZE = 640

# COS-LR 训练的置信度阈值
_CONF_THRESHOLD = 0.25
_IOU_THRESHOLD = 0.7


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


def _nms(boxes, scores, iou_threshold):
    """纯 NumPy NMS (无 torch 依赖)"""
    if len(boxes) == 0:
        return []
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= iou_threshold)[0]
        order = order[inds + 1]
    return keep


class DetectionEngine:
    """
    BPU YOLOv8 目标检测推理引擎 — 路口检测专用
    """

    def __init__(self, model_path: str = _DEFAULT_MODEL,
                 input_size: int = _INPUT_SIZE,
                 conf_threshold: float = _CONF_THRESHOLD,
                 iou_threshold: float = _IOU_THRESHOLD):
        """
        :param model_path: BPU .bin 模型路径
        :param input_size: 模型输入正方形边长 (默认 640)
        :param conf_threshold: 置信度阈值
        :param iou_threshold: NMS IoU 阈值
        """
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold

        try:
            from hobot_dnn import pyeasy_dnn as dnn
        except ImportError as e:
            raise ImportError(
                "缺少 hobot_dnn 依赖，请确认在 RDK X5 环境上运行。"
            ) from e

        models = dnn.load(model_path)
        self.model = models[0]

    def inference(self, frame: np.ndarray, debug_timing: bool = False) -> list:
        """
        对单帧 BGR 图像进行 BPU 推理，返回检测框列表。

        :param frame: BGR 格式 numpy 数组 (H, W, 3)
        :param debug_timing: 是否打印各阶段耗时
        :return: list of dict — [{"class": int, "confidence": float, "bbox": [x1,y1,x2,y2]}, ...]
                 坐标均为像素坐标(对应原始 frame 尺寸)
        """
        import time as _time
        _t = {}
        _t0 = _time.time()

        if frame is None or frame.size == 0:
            return []

        h_orig, w_orig = frame.shape[:2]

        # 1) Resize 到模型输入尺寸
        img_resized = cv2.resize(frame, (self.input_size, self.input_size))
        _t["resize"] = (_time.time() - _t0) * 1000

        # 2) BGR -> NV12
        _t1 = _time.time()
        nv12_data = _bgr2nv12(img_resized)
        _t["bgr2nv12"] = (_time.time() - _t1) * 1000

        # 3) BPU 前向推理
        _t2 = _time.time()
        outputs = self.model.forward([nv12_data])
        preds = outputs[0].buffer  # shape: [1, 14, 8400]
        _t["bpu_forward"] = (_time.time() - _t2) * 1000

        # 4) 后处理: 解码 YOLOv8 输出 → 框 + NMS
        _t3 = _time.time()
        detections = self._postprocess(preds, w_orig, h_orig)
        _t["postprocess"] = (_time.time() - _t3) * 1000

        _t["total"] = (_time.time() - _t0) * 1000

        if debug_timing:
            print(f"[YOLO BPU] total={_t['total']:.1f}ms | "
                  f"resize={_t['resize']:.1f} bgr2nv12={_t['bgr2nv12']:.1f} "
                  f"bpu_forward={_t['bpu_forward']:.1f} postprocess={_t['postprocess']:.1f} | "
                  f"detections={len(detections)}")

        return detections

    def _postprocess(self, preds: np.ndarray, img_w: int, img_h: int) -> list:
        """
        解码 YOLOv8 输出 → NMS → 检测框列表

        输入: preds shape [1, 14, 8400]
              preds[0, 0:4, :] = bbox (cx, cy, w, h) 相对 640×640
              preds[0, 4:, :]  = class scores
        """
        data = preds[0]  # [14, 8400]

        # 转置为 [8400, 14]
        data = data.T

        # 分离 bbox 和 class scores
        bbox_raw = data[:, :4]   # [8400, 4]  (cx, cy, w, h)
        scores_all = data[:, 4:]  # [8400, 10]

        # 每个 anchor 取最高分类分
        class_ids = np.argmax(scores_all, axis=1)
        confidences = np.max(scores_all, axis=1)

        # 置信度过滤
        mask = confidences >= self.conf_threshold
        if not np.any(mask):
            return []

        bbox_raw = bbox_raw[mask]
        confidences = confidences[mask]
        class_ids = class_ids[mask]

        # bbox 解码: cx,cy,w,h (相对 640) → x1,y1,x2,y2 (相对 640)
        boxes_640 = np.zeros_like(bbox_raw)
        boxes_640[:, 0] = bbox_raw[:, 0] - bbox_raw[:, 2] / 2  # x1
        boxes_640[:, 1] = bbox_raw[:, 1] - bbox_raw[:, 3] / 2  # y1
        boxes_640[:, 2] = bbox_raw[:, 0] + bbox_raw[:, 2] / 2  # x2
        boxes_640[:, 3] = bbox_raw[:, 1] + bbox_raw[:, 3] / 2  # y2

        # 裁剪到 [0, 640]
        boxes_640 = np.clip(boxes_640, 0, self.input_size)

        # NMS
        keep = _nms(boxes_640, confidences, self.iou_threshold)

        # 映射回原图尺寸
        scale_x = img_w / self.input_size
        scale_y = img_h / self.input_size

        detections = []
        for i in keep:
            x1, y1, x2, y2 = boxes_640[i]
            detections.append({
                "class": int(class_ids[i]),
                "confidence": float(confidences[i]),
                "bbox": [
                    int(x1 * scale_x),
                    int(y1 * scale_y),
                    int(x2 * scale_x),
                    int(y2 * scale_y),
                ],
            })

        return detections

    def get_intersection_info(self, detections: list) -> dict:
        """
        从检测结果中提取路口信息。

        用于控制逻辑：判断是否有路口、距离、方向偏置。

        :param detections: inference() 返回的检测框列表
        :return: {
            "has_intersection": bool,
            "num_objects": int,
            "classes": list[int],
            "bboxes": list,
        }
        """
        classes = [d["class"] for d in detections]
        return {
            "has_intersection": len(detections) > 0,
            "num_objects": len(detections),
            "classes": classes,
            "detections": detections,
        }
