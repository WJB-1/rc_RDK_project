# perception/algorithms/lane/builders.py
from pathlib import Path

from perception.algorithms.ipm import MathematicalIPM
from perception.models.pidinet import PiDiNetEngine
from perception.models.bisenet import SegmentationEngine
from perception.algorithms.pidinet_lane import (
    GroundIPMLanePairSelector,
    TemplateDistanceLaneSelector,
)

from .config import load_lane_config, LaneConfig
from .pipeline import LanePipeline
from .undistort import Undistorter


def _abs_model_path(path: str) -> str:
    p = Path(path)
    if p.is_absolute():
        return str(p)
    return str(Path(__file__).parents[3] / p)


def build_edge_engine(cfg: LaneConfig):
    edge_cfg = cfg.edge_cfg
    model_path = _abs_model_path(
        edge_cfg.get("model_path", "models/pidinet_small_dil_512x384_x5_bayese.bin")
    )
    return PiDiNetEngine(
        model_path=model_path,
        input_size=(cfg.input_width, cfg.input_height),
        edge_threshold=float(edge_cfg.get("threshold", 0.25)),
        angle_tol_deg=float(edge_cfg.get("angle_tol_deg", 3.0)),
        normal_dist_tol=float(edge_cfg.get("normal_dist_tol", 8.0)),
    )


def build_semantic_engine(cfg: LaneConfig):
    sem_cfg = cfg.semantic_gate
    if not sem_cfg.get("enabled", False) or not sem_cfg.get("available", True):
        return None
    model_path = _abs_model_path(sem_cfg.get("model_path", "models/bisenetv2_lane_x5.bin"))
    return SegmentationEngine(
        model_path=model_path,
        input_size=int(sem_cfg.get("input_size", 512)),
        target_class=int(sem_cfg.get("target_class", 1)),
    )


def build_ipm(cfg: LaneConfig) -> MathematicalIPM:
    """BEV 用第一套外参（ipm_camera）。"""
    c = cfg.ipm_camera
    return MathematicalIPM(
        img_w=c.img_w, img_h=c.img_h,
        focal_length_mm=c.focal_length_mm,
        pixel_size_mm=c.pixel_size_mm,
        fx_px=c.fx_px, fy_px=c.fy_px,
        cx_px=c.cx_px, cy_px=c.cy_px,
        camera_height_mm=c.camera_height_mm,
        pitch_deg=c.pitch_deg,
        canvas_w=cfg.canvas_w,
        canvas_h=cfg.canvas_h,
        pixel_per_mm=cfg.pixel_per_mm,
        blind_spot_mm=cfg.blind_spot_mm,
    )


def build_selector(cfg: LaneConfig, edge_engine, ipm, semantic_gate_enabled: bool):
    edge_cfg = cfg.edge_cfg
    is_template = edge_cfg.get("pair_selector", "template_distance") == "template_distance"

    c = cfg.ipm_camera
    common_kwargs = dict(
        input_size=(cfg.input_width, cfg.input_height),
        calibration_size=(c.img_w, c.img_h),
        fx_px=c.fx_px, fy_px=c.fy_px,
        cx_px=c.cx_px, cy_px=c.cy_px,
        camera_height_mm=c.camera_height_mm,
        pitch_deg=c.pitch_deg,
        canvas_w=cfg.canvas_w, canvas_h=cfg.canvas_h,
        pixel_per_mm=cfg.pixel_per_mm,
        blind_spot_mm=cfg.blind_spot_mm,
        lane_width_mm=cfg.lane_width_mm,
        vehicle_geometry=cfg.robot_geometry,
        pair_profiles=edge_cfg.get("pair_profiles"),
        debug_pair_profile=edge_cfg.get("debug_pair_profile", "auto"),
        semantic_gate=semantic_gate_enabled,
        semantic_min_coverage=float(cfg.semantic_gate.get("min_coverage", 0.01)),
        semantic_line_thickness=int(cfg.semantic_gate.get("line_thickness", 3)),
    )

    # 显式 if/else，避免 Pylance 对 **kwargs 的类型推断失败
    if is_template:
        selector: GroundIPMLanePairSelector = TemplateDistanceLaneSelector(
            **common_kwargs,
            ground_camera=cfg.ground_camera,
        )
    else:
        selector = GroundIPMLanePairSelector(**common_kwargs)

    if cfg.template_x_at_ref_mm:
        selector.template_x_at_ref_mm = dict(cfg.template_x_at_ref_mm)
    if cfg.identity_weights:
        selector.identity_weights = dict(cfg.identity_weights)
    return selector


def build_lane_pipeline(settings: dict) -> LanePipeline:
    cfg = load_lane_config(settings)
    undistorter = Undistorter(cfg.undistort_camera)
    edge_engine = build_edge_engine(cfg)
    semantic_engine = build_semantic_engine(cfg)
    semantic_gate_enabled = (
        semantic_engine is not None and cfg.semantic_gate.get("enabled", False)
    )
    ipm = build_ipm(cfg)
    selector = build_selector(cfg, edge_engine, ipm, semantic_gate_enabled)
    return LanePipeline(
        cfg=cfg,
        undistorter=undistorter,
        edge_engine=edge_engine,
        semantic_engine=semantic_engine,
        selector=selector,
        ipm=ipm,
        semantic_gate_enabled=semantic_gate_enabled,
        settings=settings,
    )