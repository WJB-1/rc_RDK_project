# Perception Algorithms Hard Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove legacy perception pipeline and root algorithm paths while preserving `test/test_correct` on the new domain layout.

**Architecture:** `algorithms.lane` owns the lane pipeline and tracker; `algorithms.core` owns technical utilities; `algorithms.observation` owns observation algorithms. No old-path re-export modules remain.

**Tech Stack:** Python, pytest, NumPy, OpenCV.

**Spec:** `docs/superpowers/specs/2026-09-20-perception-algorithms-hard-migration-design.md`

## Global Constraints

- Remove `perception/pipelines/` without compatibility shims.
- Maintain only `test/test_correct`; `main.py` may remain broken.
- Do not alter lane control calculations.

---

### Task 1: Migrate lane public imports

**Files:**
- Modify: `perception/models/pidinet.py`, `perception/algorithms/lane/builders.py`, `perception/algorithms/lane/pipeline.py`
- Modify: `test/test_correct/test_pidinet_lane.py`, `test/test_correct/test_vision_preview.py`, `test/test_correct/runner.py`, `test/test_correct/offline_runner.py`, `test/test_correct/vision_smoke.py`, `test/test_correct/bench_lane.py`

- [ ] Replace `pidinet_lane` imports with their concrete `lane.*` modules.
- [ ] Move the tracker import to `perception.algorithms.lane.tracker`.
- [ ] Run lane and preview tests.

### Task 2: Move shared and observation modules

**Files:**
- Create: `perception/algorithms/core/__init__.py`, `perception/algorithms/observation/__init__.py`
- Move: `ipm.py`, `mask_utils.py`, `timing.py`, `crossroad_detect.py`, `crossroad_seg.py`, `obstacle_detect.py`, `culvert_detect.py`, `quality_gate.py`

- [ ] Update imports inside moved files and surviving lane modules.
- [ ] Run compile and import checks for new paths.

### Task 3: Remove legacy orchestration

**Files:**
- Delete: `perception/pipelines/`, `perception/algorithms/lane_analyzer.py`, obsolete root path modules
- Modify: `perception/__init__.py`, `test/test_correct/test_perception_compat.py`

- [ ] Remove exports and tests that require old paths.
- [ ] Add test coverage for new tracker and lane utility paths.
- [ ] Confirm old paths are absent from `test/test_correct`.

### Task 4: Verify and publish

- [ ] Run `pytest test/test_correct -q` and compile checks.
- [ ] Inspect diff and stage only migration files.
- [ ] Commit and push the migration to `origin`.
