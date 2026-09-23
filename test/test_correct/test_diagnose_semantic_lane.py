import sys
import unittest
from pathlib import Path


TEST_ROOT = Path(__file__).resolve().parent
if str(TEST_ROOT) not in sys.path:
    sys.path.insert(0, str(TEST_ROOT))


class SemanticDiagnosisTests(unittest.TestCase):
    def test_diagnostic_record_flattens_gate_and_timing_values(self):
        from diagnose_semantic_lane import build_record

        record = build_record(
            "frame.jpg",
            process_ms=42.5,
            mask_coverage=0.125,
            lane_state={"lane_method": "semantic_boundary", "pid_error_mm": 8.0},
            capture={"semantic_lane": {"accepted": False, "fallback_reason": "no_pair",
                                        "image_lines": [{}, {}], "bev_lines": [{}]},
                     "metrics": {"template_confidence": None}},
            timing={"stages_ms": {"tracker.semantic_inference": 31.0}},
        )

        self.assertEqual(record["gate_accepted"], False)
        self.assertEqual(record["gate_reason"], "no_pair")
        self.assertEqual(record["image_line_count"], 2)
        self.assertEqual(record["tracker.semantic_inference_ms"], 31.0)
