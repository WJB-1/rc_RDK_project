# Lane Refactor Compatibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the `test/test_correct` lane-following entry points without reverting the new `perception.algorithms.lane` implementation.

**Architecture:** Keep the new lane subpackage as the sole implementation. Restore only thin package, tracker, and diagnostics compatibility boundaries required by existing test and runner consumers.

**Tech Stack:** Python 3.13, pytest, NumPy, OpenCV.

**Spec:** User-approved in-chat design, 2026-09-20.

## Global Constraints

- Do not restore the removed monolithic lane implementation.
- Preserve unrelated user work in the dirty worktree.
- Verify with focused compatibility tests, then `pytest test/test_correct -q`.

---

### Task 1: Break import cycle and restore package imports

**Files:**
- Modify: `perception/algorithms/lane/__init__.py`
- Modify: `perception/pipelines/__init__.py`
- Test: `test/test_correct/test_perception_compat.py`

- [ ] Write an import regression test.
- [ ] Run it and observe the circular-import failure.
- [ ] Remove eager implementation imports from package initializers.
- [ ] Run the focused test and verify import succeeds.

### Task 2: Restore LaneTracker public boundary

**Files:**
- Modify: `perception/algorithms/lane_tracker.py`
- Modify: `perception/pipelines/lane_tracker.py`
- Test: `test/test_correct/test_perception_compat.py`

- [ ] Write tests for exposed engines, selector, IPM, width, semantic state, and the undistort helper.
- [ ] Run them and observe missing attributes.
- [ ] Add direct read-only proxies and semantic-gate forwarding.
- [ ] Run focused tests and verify the runner contract.

### Task 3: Restore diagnostics import boundary

**Files:**
- Create: `perception/diagnostics/lane_debug_viz.py`
- Test: `test/test_correct/test_perception_compat.py`

- [ ] Write a preview dependency import test.
- [ ] Run it and observe the missing module failure.
- [ ] Add a compatibility module that exposes current diagnostic builders.
- [ ] Run the focused test and verify import succeeds.

### Task 4: Verify lane-following integration

**Files:**
- Test: `test/test_correct`

- [ ] Run the full `test/test_correct` suite.
- [ ] Run the offline image smoke test when model runtime is available.
- [ ] Review the final diff for unintended files.
- [ ] Commit only the compatibility repair files.
