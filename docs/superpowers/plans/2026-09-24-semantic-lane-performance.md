# Semantic Lane Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce semantic lane processing latency without changing accepted lane pairs, ground-frame yaw, or offset results.

**Architecture:** Keep BPU inference unchanged and remove CPU-side duplicate work around it. Produce only lightweight per-frame debug data, render the selected Web view on demand at a limited refresh rate, and constrain segmentation postprocessing to the road component and useful boundary region.

**Tech Stack:** Python, NumPy, OpenCV, Flask, unittest/pytest, Horizon BPU runtime.

**Spec:** User-approved performance scope from the 2026-09-24 debugging session.

## Global Constraints

- Preserve `new_ground_pair` yaw and offset calculation.
- Preserve semantic pair scoring and distance-gate semantics.
- Keep Web diagnostics available at a maximum refresh rate of 5 FPS.
- Do not modify BPU model files or UART protocol behavior.

---

### Task 1: Remove Duplicate Component Cleaning

**Files:**
- Modify: `perception/algorithms/lane/pipeline.py`
- Modify: `perception/algorithms/lane/semantic_lane.py`
- Test: `test/test_correct/test_semantic_lane.py`

**Interfaces:**
- Consumes: raw binary mask from `YoloRoadSegmentationEngine.inference()`.
- Produces: `SemanticLaneDetector.analyze()` result containing the single cleaned ground component.

- [ ] Write a failing test proving component cleaning runs once per semantic frame.
- [ ] Run the focused test and confirm the duplicate-cleaning assertion fails.
- [ ] Pass the raw mask directly to `SemanticLaneDetector.analyze()` and reuse `result["clean_mask"]`.
- [ ] Run semantic pipeline tests and confirm lane outputs are unchanged.

### Task 2: Reduce YOLO Mask Decode Work

**Files:**
- Modify: `perception/models/yolo_road_seg.py`
- Test: `test/test_correct/test_yolo_road_seg.py`

**Interfaces:**
- Consumes: prediction tensor and prototype tensor.
- Produces: the same full-resolution binary union mask and detection metadata.

- [ ] Write equivalence tests for cropped prototype-space mask composition.
- [ ] Confirm tests fail against the per-detection full-frame resize implementation.
- [ ] Combine accepted masks in prototype space and resize the union once.
- [ ] Run decoder tests and compare exact binary output for single and multiple detections.

### Task 3: Constrain Boundary Detection Work

**Files:**
- Modify: `perception/algorithms/lane/semantic_lane.py`
- Test: `test/test_correct/test_semantic_lane.py`

**Interfaces:**
- Consumes: cleaned ground-connected component.
- Produces: boundary points and merged Hough segments in original image coordinates.

- [ ] Write tests proving empty rows and the component bounding box do not change line coordinates.
- [ ] Confirm the new ROI behavior test fails.
- [ ] Scan only the selected component bounding rows and run Hough on its padded bounding ROI.
- [ ] Reapply the ROI offset before downstream projection and run semantic tests.

### Task 4: Render Web Views On Demand

**Files:**
- Modify: `perception/algorithms/lane/pipeline.py`
- Modify: `test/test_correct/runner.py`
- Modify: `test/test_correct/vision_preview.py`
- Test: `test/test_correct/test_vision_preview.py`

**Interfaces:**
- Consumes: raw frame, masks, lane state, and compact debug primitives.
- Produces: only the view requested through `/api/vision?view=...`.

- [ ] Write tests proving frame processing does not render unrequested debug views.
- [ ] Confirm the eager-rendering test fails.
- [ ] Store compact capture data and move view construction into request-time rendering.
- [ ] Cache each rendered view by frame ID and run preview tests.

### Task 5: Limit Debug Refresh To Five FPS

**Files:**
- Modify: `test/test_correct/runner.py`
- Test: `test/test_correct/test_vision_preview.py`

**Interfaces:**
- Consumes: continuously updated inference results.
- Produces: Web preview snapshots no more frequently than every 200 ms.

- [ ] Write a failing test for the 200 ms preview update interval.
- [ ] Confirm rapid updates are currently accepted.
- [ ] Skip debug snapshot construction until the interval elapses while retaining latest lane telemetry.
- [ ] Run runner and preview tests, then benchmark core and debug-enabled paths.

### Task 6: Verify Correctness And Performance

**Files:**
- Update only if applicable: project engineering-debt records.

**Interfaces:**
- Consumes: optimized semantic pipeline.
- Produces: test evidence and device-side timing fields compatible with existing Web diagnostics.

- [ ] Run focused decoder, semantic detector, pipeline, runner, and preview tests.
- [ ] Run `git diff --check` on touched files.
- [ ] Record PC microbenchmarks without claiming RDK device latency.
- [ ] Review engineering-debt records and document only materially affected debt items.
