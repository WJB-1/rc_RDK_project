"""视觉 Profile 加载与车身共享几何注入。"""

from pathlib import Path

from config.vehicle import get_vehicle_profile


_PROFILE_PATH = Path(__file__).with_name("vision_profile.yaml")


def apply_vision_profile(settings):
    """Overlay perception-owned settings from the repository vision profile."""
    import copy

    merged = copy.deepcopy(settings or {})

    def merge(base, overlay):
        for key, value in overlay.items():
            if isinstance(value, dict) and isinstance(base.get(key), dict):
                merge(base[key], value)
            else:
                base[key] = copy.deepcopy(value)

    merge(merged, load_vision_profile())
    return merged


def load_vision_profile():
    import yaml

    with _PROFILE_PATH.open("r", encoding="utf-8") as profile_file:
        profile = yaml.safe_load(profile_file) or {}
    if "vehicle_geometry" in profile:
        raise ValueError("vision profile must not define vehicle_geometry")
    return profile


def build_perception_settings():
    """在 Perception 组合根按需注入共享车身几何。"""

    settings = load_vision_profile()
    settings["vehicle_geometry"] = get_vehicle_profile().as_geometry()
    return settings
