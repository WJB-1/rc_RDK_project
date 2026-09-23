#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LaneTracker 离线耗时基准测试（不输出图像，不做图传）

用法：
    python bench_lane.py
    python bench_lane.py --img-dir /path/to/imgs --repeat 2 --warmup 3
    python bench_lane.py --skip-capture        # 跳过 capture_debug（Web 端可视化）

输出：
    每张图片总耗时 + 所有阶段耗时的均值和最大值。
"""

import argparse
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]   # test/test_correct -> test -> robocup_rescue_brain
sys.path.insert(0, str(PROJECT_ROOT))


DEFAULT_IMG_DIR = PROJECT_ROOT / "perception" / "test_img"
DEFAULT_SETTINGS = PROJECT_ROOT / "config" / "settings.yaml"

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def load_settings(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        settings = yaml.safe_load(f) or {}
    from perception.profiles import apply_vision_profile
    return apply_vision_profile(settings)


def iter_images(img_dir: Path):
    for p in sorted(img_dir.iterdir()):
        if p.is_file() and p.suffix.lower() in IMAGE_EXTS:
            yield p


def main() -> int:
    parser = argparse.ArgumentParser(description="LaneTracker timing benchmark")
    parser.add_argument("--img-dir", type=Path, default=DEFAULT_IMG_DIR)
    parser.add_argument("--settings", type=Path, default=DEFAULT_SETTINGS)
    parser.add_argument("--repeat", type=int, default=1,
                        help="每张图重复处理次数，默认 1")
    parser.add_argument("--warmup", type=int, default=2,
                        help="预热帧数，默认 2（用第一张图）")
    parser.add_argument("--skip-capture", action="store_true",
                        help="跳过 tracker.capture_debug（测核心链路耗时）")
    parser.add_argument("--mode", choices=("template", "semantic", "auto"), default="auto",
                        help="车道链路：template/semantic/auto，默认 auto")
    parser.add_argument("--capture-debug", action="store_true",
                        help="开启与 Web 调试端一致的 debug_capture")
    args = parser.parse_args()

    if not args.img_dir.exists():
        print(f"[error] image dir not found: {args.img_dir}")
        return 1
    if not args.settings.exists():
        print(f"[error] settings not found: {args.settings}")
        return 1

    images = list(iter_images(args.img_dir))
    if not images:
        print(f"[error] no images in {args.img_dir}")
        return 1
    print(f"images: {len(images)}  ({args.img_dir})")

    # 关掉 timing 文件写入，避免日志膨胀；但仍能通过 get_frame_timings() 拿本帧数据
    settings = load_settings(args.settings)
    settings.setdefault("debug", {})["timing_enabled"] = False

    from perception.algorithms.lane.tracker import LaneTracker
    from perception.algorithms.core.timing import get_frame_timings

    print("initializing LaneTracker ...")
    t_init = time.perf_counter()
    tracker = LaneTracker(settings)
    tracker.set_semantic_lane_mode(args.mode)
    print(f"init done in {(time.perf_counter() - t_init) * 1000:.0f} ms")

    # 可选：跳过 capture_debug（不显示在 Web 上时）
    if args.skip_capture:
        tracker.set_debug_capture_enabled(False)
        print("capture_debug disabled")
    elif args.capture_debug:
        tracker.set_debug_capture_enabled(True)
        print("capture_debug enabled")

    # 预热
    first = cv2.imread(str(images[0]))
    if first is None:
        print(f"[error] failed to read first image: {images[0]}")
        return 1
    for _ in range(args.warmup):
        tracker.process(first)
    print(f"warmup done ({args.warmup} frames)")

    # 主循环
    per_image_total = {}
    per_image_stages = {}
    all_stages = {}
    method_counts = {}
    template_diagnostics = []

    for img_path in images:
        frame = cv2.imread(str(img_path))
        if frame is None:
            print(f"  [skip] failed to read: {img_path.name}")
            continue

        for _ in range(args.repeat):
            t0 = time.perf_counter()
            try:
                tracker.process(frame)
            except Exception as error:
                print(f"  [err] {img_path.name}: {error}")
                continue
            total_ms = (time.perf_counter() - t0) * 1000.0

            lane_method = (tracker.last_lane_state or {}).get("lane_method", "unknown")
            method_counts[lane_method] = method_counts.get(lane_method, 0) + 1
            diagnostics = getattr(tracker.lane_selector, "_last_template_match_diagnostics", {}) or {}
            if diagnostics:
                template_diagnostics.append(diagnostics)

            per_image_total.setdefault(img_path.name, []).append(total_ms)
            stage_map = per_image_stages.setdefault(img_path.name, {})
            for name, ms in get_frame_timings():
                stage_map.setdefault(name, []).append(ms)
                all_stages.setdefault(name, []).append(ms)

    if not per_image_total:
        print("[error] no successful runs")
        return 1

    all_totals = [ms for v in per_image_total.values() for ms in v]
    print()
    print("=== lane methods ===")
    for method, count in sorted(method_counts.items()):
        print(f"  {method:<36s} {count}")

    if template_diagnostics:
        print()
        print("=== template matching diagnostics ===")
        for key in ("input_line_count", "evaluated_combinations", "accepted_combinations",
                    "elapsed_ms", "avg_combination_us"):
            values = [float(item[key]) for item in template_diagnostics]
            print(f"  {key:<30s} mean {statistics.mean(values):10.2f}  max {max(values):10.2f}")
        print("  parallel_group_sizes:", [item["parallel_group_sizes"] for item in template_diagnostics])

    def stage_mean(name):
        values = all_stages.get(name, ())
        return statistics.mean(values) if values else 0.0

    print()
    print("[PROFILE]")
    for label, stage_name in (
        ("BEV projection", "lane.project_lines_bev"),
        ("ground projection", "lane.source_to_new_ground"),
        ("line params", "lane.line_params"),
        ("parallel grouping", "lane.parallel_groups"),
        ("candidate delta", "lane.candidate_delta"),
        ("template subset DP", "lane.template_subset_dp"),
        ("template matching", "lane.template_match"),
        ("confidence", "lane.compute_confidence"),
        ("yaw/offset", "lane.yaw_offset"),
    ):
        print(f"{label:<21}: {stage_mean(stage_name):7.2f} ms")
    print("--------------------------------")
    print(f"analyze total        : {stage_mean('lane.analyze_total'):7.2f} ms")

    print()
    print(f"=== per-image total (n={len(all_totals)}) ===")
    print(f"  mean   {statistics.mean(all_totals):8.1f} ms")
    print(f"  median {statistics.median(all_totals):8.1f} ms")
    print(f"  min    {min(all_totals):8.1f} ms")
    print(f"  max    {max(all_totals):8.1f} ms")

    print()
    print("=== per-image totals ===")
    for name in sorted(per_image_total.keys()):
        tot = per_image_total[name]
        print(f"  {name:<44s}  mean {statistics.mean(tot):7.1f} ms  "
              f"min {min(tot):7.1f}  max {max(tot):7.1f}")

    print()
    print("=== stage means (across all frames, sorted by mean) ===")
    rows = []
    for name, ms_list in all_stages.items():
        rows.append((name, statistics.mean(ms_list), max(ms_list), len(ms_list)))
    rows.sort(key=lambda r: -r[1])

    # 顶部子阶段会被父阶段重复计算，标注一下
    for name, mean_ms, max_ms, n in rows:
        print(f"  {name:<40s}  mean {mean_ms:8.2f} ms  max {max_ms:8.2f} ms  n={n}")

    # 叶子节点求和（去掉父级嵌套）
    parent_stages = {
        "tracker.edge_inference": {
            "edge.resize", "edge.enhance", "edge.bgr2nv12", "edge.bpu_forward",
            "edge.probability", "edge.threshold", "edge.close", "edge.thin",
            "edge.hough", "edge.merge", "edge.connected_components",
        },
        "tracker.selector_analyze": {
            "lane.project_lines_bev", "lane.template_match",
            "lane.source_to_new_ground", "lane.fit_new_ground",
            "lane.theta_offset", "lane.new_ground_to_image",
        },
    }
    child_names = set()
    for kids in parent_stages.values():
        child_names |= kids

    print()
    print("=== leaf-stage summary (parents excluded) ===")
    leaf_mean_sum = 0.0
    for name, mean_ms, max_ms, n in rows:
        if name in child_names:
            continue
        if name.startswith("tracker."):
            print(f"  {name:<40s}  mean {mean_ms:8.2f} ms")
            leaf_mean_sum += mean_ms
    print(f"  {'sum of top-level tracker.* leaves':<40s}  mean {leaf_mean_sum:8.2f} ms")

    return 0


if __name__ == "__main__":
    sys.exit(main())
