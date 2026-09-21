# perception/algorithms/lane/config.py
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraConfig:
    """单套相机配置：内参 + 外参。

    车道检测使用两套：
      - ipm_camera   : 图像 → BEV，用于车道线选择（pair / template match）
      - ground_camera: 图像 → 地面系，用于偏航角、横向偏移量的最终计算

    内参可共享（同一物理相机），外参必须独立标定。
    """
    # ---- 内参 ----
    img_w: int
    img_h: int
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float
    focal_length_mm: float = 2.8
    pixel_size_mm: float = 0.003

    # ---- 外参 ----
    camera_height_mm: float = 190.0
    pitch_deg: float = 40.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class LaneConfig:
    # 两套相机
    ipm_camera: CameraConfig
    ground_camera: CameraConfig

    # IPM 画布（只属于 BEV，与相机解耦）
    canvas_w: int
    canvas_h: int
    pixel_per_mm: float
    blind_spot_mm: float

    # 网络输入尺寸
    input_width: int
    input_height: int

    # 业务参数
    lane_width_mm: float
    robot_geometry: dict
    template_x_at_ref_mm: dict
    identity_weights: dict
    edge_cfg: dict
    matching_cfg: dict
    confidence_cfg: dict
    bev_cfg: dict
    semantic_gate: dict
    undistort_camera: dict


# ----------------------------------------------------------------
# 加载
# ----------------------------------------------------------------
def _load_camera(section: dict, defaults: dict) -> CameraConfig:
    """从 YAML 的一个 dict section 构造 CameraConfig。defaults 必须含所有键。"""
    def g(k):
        return section[k] if k in section else defaults[k]

    return CameraConfig(
        img_w=int(g("img_w")),
        img_h=int(g("img_h")),
        fx_px=float(g("fx_px")),
        fy_px=float(g("fy_px")),
        cx_px=float(g("cx_px")),
        cy_px=float(g("cy_px")),
        focal_length_mm=float(g("focal_length_mm")),
        pixel_size_mm=float(g("pixel_size_mm")),
        camera_height_mm=float(g("camera_height_mm")),
        pitch_deg=float(g("pitch_deg")),
        roll_deg=float(g("roll_deg")),
        yaw_deg=float(g("yaw_deg")),
    )


def _legacy_base_intrinsics(legacy_ipm: dict) -> dict:
    """旧 math_ipm 兼容默认值。"""
    return {
        "img_w": int(legacy_ipm.get("img_w", 1920)),
        "img_h": int(legacy_ipm.get("img_h", 1080)),
        "fx_px": float(legacy_ipm.get("fx_px", 1131.5665939049366)),
        "fy_px": float(legacy_ipm.get("fy_px", 1131.5665939049366)),
        "cx_px": float(legacy_ipm.get("cx_px", 951.0970902161366)),
        "cy_px": float(legacy_ipm.get("cy_px", 553.6989838702775)),
        "focal_length_mm": float(legacy_ipm.get("focal_length_mm", 2.8)),
        "pixel_size_mm": float(legacy_ipm.get("pixel_size_mm", 0.003)),
        "camera_height_mm": float(legacy_ipm.get("camera_height_mm", 190.0)),
        "pitch_deg": float(legacy_ipm.get("pitch_deg", 40.0)),
        "roll_deg": 0.0,
        "yaw_deg": 0.0,
    }


def load_lane_config(settings: dict) -> LaneConfig:
    lane_cfg = dict(settings.get("lane_detection", {}) or {})
    legacy_ipm = dict(settings.get("math_ipm", {}) or {})
    legacy_edge = dict(settings.get("edge_detection", {}) or {})
    legacy_track = dict(settings.get("track", {}) or {})
    legacy_vehicle = dict(settings.get("vehicle_geometry", {}) or {})

    base_defaults = _legacy_base_intrinsics(legacy_ipm)

    # ---- 第一套：IPM 相机（lane_detection.ipm_camera > math_ipm） ----
    ipm_section = dict(lane_cfg.get("ipm_camera", {}) or {})
    if not ipm_section:
        ipm_section = dict(legacy_ipm)
    ipm_camera = _load_camera(ipm_section, base_defaults)

    # ---- 第二套：地面相机（lane_detection.ground_camera > ipm_camera 兜底） ----
    ground_section = dict(lane_cfg.get("ground_camera", {}) or {})
    if not ground_section:
        # 向后兼容：没有单独配置时，退化为 ipm_camera
        ground_section = dict(ipm_section)
    ground_defaults = {
        "img_w": ipm_camera.img_w,
        "img_h": ipm_camera.img_h,
        "fx_px": ipm_camera.fx_px,
        "fy_px": ipm_camera.fy_px,
        "cx_px": ipm_camera.cx_px,
        "cy_px": ipm_camera.cy_px,
        "focal_length_mm": ipm_camera.focal_length_mm,
        "pixel_size_mm": ipm_camera.pixel_size_mm,
        "camera_height_mm": ipm_camera.camera_height_mm,
        "pitch_deg": ipm_camera.pitch_deg,
        "roll_deg": 0.0,
        "yaw_deg": 0.0,
    }
    ground_camera = _load_camera(ground_section, ground_defaults)

    # ---- IPM 画布（只属于 BEV） ----
    canvas_w = int(ipm_section.get("canvas_w", 400))
    canvas_h = int(ipm_section.get("canvas_h", 400))
    pixel_per_mm = float(ipm_section.get("pixel_per_mm", 0.5))
    blind_spot_mm = float(ipm_section.get("blind_spot_mm", 90.0))

    # ---- 网络输入尺寸 ----
    input_size = lane_cfg.get("input_size")
    if input_size is None:
        input_width = int(legacy_edge.get("input_width", 512))
        input_height = int(legacy_edge.get("input_height", 384))
    else:
        input_width, input_height = map(int, input_size)

    return LaneConfig(
        ipm_camera=ipm_camera,
        ground_camera=ground_camera,
        canvas_w=canvas_w,
        canvas_h=canvas_h,
        pixel_per_mm=pixel_per_mm,
        blind_spot_mm=blind_spot_mm,
        input_width=input_width,
        input_height=input_height,
        lane_width_mm=float(
            lane_cfg.get("lane_width_mm", legacy_track.get("lane_width_mm", 200.0))
        ),
        robot_geometry=dict(lane_cfg.get("robot_geometry", legacy_vehicle) or {}),
        template_x_at_ref_mm=dict(lane_cfg.get("template_x_at_ref_mm", {}) or {}),
        identity_weights=dict(lane_cfg.get("identity_weights", {}) or {}),
        edge_cfg=dict(legacy_edge or {}),
        matching_cfg=dict(lane_cfg.get("matching", {}) or {}),
        confidence_cfg=dict(lane_cfg.get("confidence", {}) or {}),
        bev_cfg=dict(lane_cfg.get("bev", {}) or {}),
        semantic_gate=dict(legacy_edge.get("semantic_gate", {}) or {}),
        undistort_camera=dict(
            lane_cfg.get("undistort_camera",
                        legacy_ipm.get("camera_calibration", {})) or {}
        ),
    )