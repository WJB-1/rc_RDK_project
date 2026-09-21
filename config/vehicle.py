"""跨 Navigation、Motion 与 Perception 的只读车身参数。"""

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class VehicleProfile:
    body_length_mm: float
    body_width_mm: float
    camera_forward_of_body_front_mm: float
    camera_width_mm: float
    camera_length_mm: float
    camera_lateral_offset_mm: float

    def as_geometry(self):
        return {
            "body_length_mm": self.body_length_mm,
            "body_width_mm": self.body_width_mm,
            "camera_forward_of_body_front_mm": self.camera_forward_of_body_front_mm,
            "camera_width_mm": self.camera_width_mm,
            "camera_length_mm": self.camera_length_mm,
            "camera_lateral_offset_mm": self.camera_lateral_offset_mm,
        }


_PROFILE_PATH = Path(__file__).with_name("settings.yaml")
_profile = None


def get_vehicle_profile():
    global _profile
    if _profile is None:
        import yaml

        with _PROFILE_PATH.open("r", encoding="utf-8") as profile_file:
            geometry = (yaml.safe_load(profile_file) or {}).get("vehicle_geometry", {})
        _profile = VehicleProfile(**geometry)
    return _profile
