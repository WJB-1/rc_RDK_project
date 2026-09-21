import cv2
import numpy as np


VIEW_NAMES = {
    "overlay": "原图模板线与中心线",
    "binary": "边缘检测二值化结果",
    "hough": "原图融合 Hough 线段",
    "lane_bev": "平行坐标系模板匹配",
    "ground_bev": "地面坐标系车道与中心线",
}


def _as_bgr(image):
    if image is None:
        return np.zeros((360, 640, 3), dtype=np.uint8)
    image = np.asarray(image)
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.ndim == 3 and image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


def _resize_for_transfer(image, max_size):
    max_width, max_height = max_size
    height, width = image.shape[:2]
    scale = min(max_width / max(width, 1), max_height / max(height, 1), 1.0)
    if scale < 1.0:
        return cv2.resize(
            image,
            (max(1, round(width * scale)), max(1, round(height * scale))),
            interpolation=cv2.INTER_AREA,
        )
    return image


def _draw_merged_lines(image, lines, source_size=None):
    if source_size:
        source_width, source_height = source_size
        scale_x = image.shape[1] / float(source_width)
        scale_y = image.shape[0] / float(source_height)
    else:
        scale_x = scale_y = 1.0
    for line in lines or []:
        if isinstance(line, dict):
            start = (int(round(line["x1"] * scale_x)), int(round(line["y1"] * scale_y)))
            end = (int(round(line["x2"] * scale_x)), int(round(line["y2"] * scale_y)))
        else:
            points = np.asarray(line, dtype=np.float64).reshape(2, 2)
            start = tuple(np.round(points[0] * (scale_x, scale_y)).astype(int))
            end = tuple(np.round(points[1] * (scale_x, scale_y)).astype(int))
        cv2.line(image, start, end, (0, 0, 255), 2, cv2.LINE_AA)
    return image


def _fallback_view(view_name, raw_image, clean_mask, bev_mask, original_view, diagnostics):
    if view_name == "overlay":
        return original_view if original_view is not None else raw_image
    if view_name == "binary":
        return diagnostics.get("binary", clean_mask)
    if view_name == "hough":
        return _draw_merged_lines(
            _as_bgr(raw_image), diagnostics.get("merged_lines"), diagnostics.get("source_size")
        )
    if view_name == "lane_bev":
        return bev_mask
    return None


def render_preview(view_name, raw_image, clean_mask, bev_mask, lane_state, camera_pitch_deg=40.0,
                   physical_track_width_mm=450.0, offset_mm=0.0, process_ms=0.0,
                   max_size=(640, 480), jpeg_quality=72, original_view=None, diagnostics=None,
                   timing=None):
    if view_name not in VIEW_NAMES:
        raise ValueError(f"unknown vision view: {view_name}")

    diagnostics = diagnostics or {}
    debug_capture = diagnostics.get("debug_capture") or {}
    lane_views = debug_capture.get("lane_views") or {}
    image = lane_views.get(view_name)
    if image is None:
        image = _fallback_view(view_name, raw_image, clean_mask, bev_mask, original_view, diagnostics)
    image = _resize_for_transfer(_as_bgr(image), max_size)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    if not ok:
        raise RuntimeError("vision preview JPEG encoding failed")
    return encoded.tobytes()
