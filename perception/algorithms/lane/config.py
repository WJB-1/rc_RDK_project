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
    semantic_lane: dict
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


def load_lane_config(settings: dict) -> LaneConfig:
    lane_cfg = dict(settings["lane_detection"])
    edge_cfg = dict(settings["edge_detection"])
    vehicle_geometry = dict(settings["vehicle_geometry"])
    ipm_section = dict(lane_cfg["ipm_camera"])
    ipm_camera = _load_camera(ipm_section, ipm_section)

    ground_section = dict(lane_cfg["ground_camera"])
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
    input_width, input_height = map(int, lane_cfg["input_size"])

    return LaneConfig(
        ipm_camera=ipm_camera,
        ground_camera=ground_camera,
        canvas_w=canvas_w,
        canvas_h=canvas_h,
        pixel_per_mm=pixel_per_mm,
        blind_spot_mm=blind_spot_mm,
        input_width=input_width,
        input_height=input_height,
        lane_width_mm=float(lane_cfg["lane_width_mm"]),
        robot_geometry=vehicle_geometry,
        template_x_at_ref_mm=dict(lane_cfg["template_x_at_ref_mm"]),
        identity_weights=dict(lane_cfg["identity_weights"]),
        edge_cfg=edge_cfg,
        matching_cfg=dict(lane_cfg["matching"]),
        confidence_cfg=dict(lane_cfg["confidence"]),
        bev_cfg=dict(lane_cfg["bev"]),
        semantic_gate=dict(edge_cfg["semantic_gate"]),
        semantic_lane=dict(lane_cfg.get("semantic_lane", {})),
        undistort_camera=dict(lane_cfg["undistort_camera"]),
    )
