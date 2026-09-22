# perception/algorithms/lane/line_detection.py
"""边缘图 → 线段：骨架化、Hough、合并、mask 转换。

常量统一从 .constants 取，模块内不再本地定义。
"""
import math

import cv2
import numpy as np

from .constants import ANGLE_TOL_DEG, NORMAL_DIST_TOL, MIN_LINE_LENGTH


def thin_binary(binary: np.ndarray) -> np.ndarray:
    binary = np.where(np.asarray(binary) > 0, 255, 0).astype(np.uint8)
    ximgproc = getattr(cv2, "ximgproc", None)
    if ximgproc is not None and hasattr(ximgproc, "thinning"):
        return ximgproc.thinning(binary, thinningType=ximgproc.THINNING_ZHANGSUEN)
    skeleton = np.zeros_like(binary)
    current = binary.copy()
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while np.any(current):
        eroded = cv2.erode(current, kernel)
        opened = cv2.dilate(eroded, kernel)
        skeleton = cv2.bitwise_or(skeleton, cv2.subtract(current, opened))
        current = eroded
    return skeleton


def postprocess_edge_probability(probability: np.ndarray, threshold: float = 0.35) -> np.ndarray:
    probability = np.asarray(probability, dtype=np.float32)
    return (probability >= float(threshold)).astype(np.uint8) * 255


def erode_edge_segments(binary: np.ndarray) -> np.ndarray:
    binary = np.where(np.asarray(binary) > 0, 255, 0).astype(np.uint8)
    return cv2.erode(binary, np.ones((4, 4), dtype=np.uint8), iterations=1)


def detect_component_centerlines(binary: np.ndarray, min_length=30.0):
    """Fit one center axis to each connected thick edge segment."""
    binary = np.where(np.asarray(binary) > 0, 255, 0).astype(np.uint8)
    label_count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    centerlines = []
    for label in range(1, label_count):
        x, y, width, height, area = stats[label]
        if area < 4:
            continue
        points = np.column_stack(np.where(labels == label))[:, ::-1].astype(np.float32)
        vx, vy, x0, y0 = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01).reshape(4)
        direction = np.array([vx, vy], dtype=np.float64)
        norm = float(np.linalg.norm(direction))
        if norm < 1e-6:
            continue
        direction /= norm
        origin = np.array([x0, y0], dtype=np.float64)
        projections = (points - origin) @ direction
        first = origin + direction * float(projections.min())
        second = origin + direction * float(projections.max())
        length = float(np.linalg.norm(second - first))
        if length < float(min_length):
            continue
        angle = math.degrees(math.atan2(float(direction[1]), float(direction[0]))) % 180.0
        centerlines.append({
            "x1": float(first[0]), "y1": float(first[1]),
            "x2": float(second[0]), "y2": float(second[1]),
            "length": length,
            "angle_deg": angle,
            "orientation": _orientation(angle),
            "component_area": int(area),
        })
    return sorted(centerlines, key=lambda line: line["length"], reverse=True)


def _line_params(line):
    x1, y1, x2, y2 = line["x1"], line["y1"], line["x2"], line["y2"]
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return None
    ux, uy = dx / length, dy / length
    if ux < 0 or (abs(ux) < 1e-9 and uy < 0):
        ux, uy = -ux, -uy
    nx, ny = -uy, ux
    return {
        "cx": (x1 + x2) * 0.5,
        "cy": (y1 + y2) * 0.5,
        "ux": ux,
        "uy": uy,
        "nx": nx,
        "ny": ny,
        "angle": math.degrees(math.atan2(uy, ux)) % 180.0,
        "length": length,
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
    }


def _can_merge(first, second, angle_tol_deg, normal_dist_tol):
    angle_delta = abs(first["angle"] - second["angle"])
    if angle_delta > 90.0:
        angle_delta = 180.0 - angle_delta
    if angle_delta > angle_tol_deg:
        return False
    first_distance = abs(
        (second["cx"] - first["cx"]) * first["nx"]
        + (second["cy"] - first["cy"]) * first["ny"]
    )
    second_distance = abs(
        (first["cx"] - second["cx"]) * second["nx"]
        + (first["cy"] - second["cy"]) * second["ny"]
    )
    return first_distance <= normal_dist_tol and second_distance <= normal_dist_tol


def _fit_line_pca(points):
    points = np.asarray(points, dtype=np.float64)
    mean = points.mean(axis=0)
    centered = points - mean
    covariance = centered.T @ centered / max(len(points), 1)
    _, vectors = np.linalg.eigh(covariance)
    direction = vectors[:, -1]
    projections = centered @ direction
    first = mean + direction * projections.min()
    second = mean + direction * projections.max()
    return float(first[0]), float(first[1]), float(second[0]), float(second[1])


def _orientation(angle):
    if angle < 20.0 or angle > 160.0:
        return "horizontal"
    if 70.0 <= angle <= 110.0:
        return "vertical"
    return "diagonal"


def merge_lines(lines, angle_tol_deg=ANGLE_TOL_DEG, normal_dist_tol=NORMAL_DIST_TOL,
                min_length=MIN_LINE_LENGTH):
    params = [item for item in (_line_params(line) for line in lines) if item is not None]
    if not params:
        return []
    parent = list(range(len(params)))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(first, second):
        first_root, second_root = find(first), find(second)
        if first_root != second_root:
            parent[second_root] = first_root

    for first_index, first in enumerate(params):
        for second_index in range(first_index + 1, len(params)):
            second = params[second_index]
            center_distance = math.hypot(first["cx"] - second["cx"], first["cy"] - second["cy"])
            bound = (first["length"] + second["length"]) * 0.5 + normal_dist_tol
            if center_distance <= bound * 4 and _can_merge(first, second, angle_tol_deg, normal_dist_tol):
                union(first_index, second_index)

    groups = {}
    for index in range(len(params)):
        groups.setdefault(find(index), []).append(index)

    merged = []
    for indices in groups.values():
        points = []
        for index in indices:
            points.extend(((params[index]["x1"], params[index]["y1"]),
                           (params[index]["x2"], params[index]["y2"])))
        if len(indices) == 1:
            first = params[indices[0]]
            x1, y1, x2, y2 = first["x1"], first["y1"], first["x2"], first["y2"]
        else:
            x1, y1, x2, y2 = _fit_line_pca(points)
        length = math.hypot(x2 - x1, y2 - y1)
        if length < min_length:
            continue
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0
        merged.append({
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
            "length": float(length),
            "angle_deg": float(angle),
            "orientation": _orientation(angle),
            "merged_from": len(indices),
        })
    return sorted(merged, key=lambda item: item["length"], reverse=True)


def detect_lines(edge_binary: np.ndarray):
    height, width = edge_binary.shape[:2]
    lines = cv2.HoughLinesP(
        edge_binary,
        rho=1,
        theta=np.pi / 180,
        threshold=30,
        minLineLength=int(max(30.0, min(width, height) * 0.08)),
        maxLineGap=max(8, int(min(width, height) * 0.02)),
    )
    if lines is None:
        return []
    result = []
    for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
        length = math.hypot(float(x2 - x1), float(y2 - y1))
        if length < MIN_LINE_LENGTH:
            continue
        angle = math.degrees(math.atan2(float(y2 - y1), float(x2 - x1))) % 180.0
        result.append({
            "x1": float(x1), "y1": float(y1), "x2": float(x2), "y2": float(y2),
            "length": length, "angle_deg": angle, "orientation": _orientation(angle),
        })
    return sorted(result, key=lambda item: item["length"], reverse=True)


def draw_lines(image: np.ndarray, lines, color=(0, 0, 255), thickness=2):
    result = image.copy()
    for line in lines:
        cv2.line(
            result,
            (round(line["x1"]), round(line["y1"])),
            (round(line["x2"]), round(line["y2"])),
            color,
            thickness,
            cv2.LINE_AA,
        )
    return result


def lines_to_mask(lines, shape, thickness=3):
    mask = np.zeros(shape[:2], dtype=np.uint8)
    for line in lines:
        cv2.line(
            mask,
            (round(line["x1"]), round(line["y1"])),
            (round(line["x2"]), round(line["y2"])),
            255,
            thickness,
            cv2.LINE_AA,
        )
    return mask


def line_mask_coverage(line, semantic_mask, thickness=3):
    if semantic_mask is None or semantic_mask.size == 0:
        return 0.0
    line_mask = np.zeros(semantic_mask.shape[:2], dtype=np.uint8)
    cv2.line(line_mask, (round(line["x1"]), round(line["y1"])),
             (round(line["x2"]), round(line["y2"])), 255, int(thickness), cv2.LINE_AA)
    line_pixels = line_mask > 0
    if not np.any(line_pixels):
        return 0.0
    covered = (np.asarray(semantic_mask) > 0) & line_pixels
    return float(np.count_nonzero(covered) / np.count_nonzero(line_pixels))
