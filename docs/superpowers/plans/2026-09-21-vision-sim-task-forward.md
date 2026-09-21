# Vision Adapter and Simulated Task Forward Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the Perception facade's lane-analysis adapter and make simulated task execution perform a 300 mm exploration drive followed by a vision-estimated drive.

**Architecture:** A small compatibility `VisionPipeline` translates the legacy tracker tuple into the immutable Perception contracts. `SimTaskPort` owns its internal motion sequence and obtains the second distance from `SimWorld`; Navigation receives only the final task completion.

**Tech Stack:** Python 3, `unittest`, existing Navigation execution contracts.

**Spec:** `docs/superpowers/specs/2026-09-21-vision-sim-task-forward-design.md`

## Global Constraints

- Internal task drives are simulation-only and must not modify NavigationStateStore.
- The first task exploration drive is exactly `300.0` mm.
- The second task drive uses a positive simulated visual distance to the next turn window.
- Invalid visual distance fails the task without calling `SimWorld.execute_task`.

---

### Task 1: Restore the lane-analysis adapter

**Files:**
- Create: `perception/pipelines/__init__.py`
- Create: `perception/pipelines/lane.py`
- Modify: `test/perception/test_pipelines.py`
- Test: `test/perception/test_facade.py`

**Interfaces:**
- Consumes: `tracker.process(frame) -> (offset_mm, is_intersection, debug_frame)` and `tracker.last_lane_state`.
- Produces: `VisionPipeline.analyze(frame, frame_id, captured_at_monotonic_ns) -> FrameAnalysis`.

- [x] **Step 1: Write failing tests** for a valid tracker result and a dropped frame result.
- [x] **Step 2: Run** `D:\anaconda\python.exe -m unittest test.perception.test_pipelines test.perception.test_facade` and confirm the missing adapter failure.
- [x] **Step 3: Implement** `VisionPipeline` with `FrameAnalysis`, `LaneMeasurement`, `VALID`, and non-valid sample conversion.
- [x] **Step 4: Re-run** the two Perception test modules and confirm success.
- [ ] **Step 5: Commit** the adapter and its tests with `fix(perception): restore lane facade adapter`.

### Task 2: Simulate task-owned forward sequence

**Files:**
- Modify: `navigation/simulation/world.py`
- Modify: `navigation/simulation/ports.py`
- Modify: `test/navigation/2_0/test_simulation_ports.py`

**Interfaces:**
- Consumes: `ExecuteTaskExecutionCommand`.
- Produces: `SimWorld.estimate_distance_to_next_turn_window_mm() -> Optional[float]` and a final `TargetCompletion`.

- [x] **Step 1: Write failing tests** asserting `[300.0, estimated_distance]` internal drives before task completion and failure for an invalid estimate.
- [x] **Step 2: Run** `D:\anaconda\python.exe -m unittest test.navigation.2_0.test_simulation_ports` and confirm the behavior is absent.
- [x] **Step 3: Implement** deterministic visual distance estimation and the ordered internal task sequence.
- [x] **Step 4: Re-run** the simulation-port tests and confirm success.
- [ ] **Step 5: Commit** the simulation behavior and tests with `feat(simulation): simulate culvert task forward sequence`.

### Task 3: Run focused regression checks

**Files:**
- Modify: `docs/superpowers/specs/2026-09-21-vision-sim-task-forward-design.md` only if evidence changes documented requirements.

- [x] **Step 1: Run** `D:\anaconda\python.exe -m unittest test.perception.test_contracts test.perception.test_pipelines test.perception.test_facade test.navigation.2_0.test_simulation_ports`.
- [x] **Step 2: Run** `D:\anaconda\python.exe -m unittest discover -s test\motion -p test_*.py`.
- [x] **Step 3: Review** `git diff --check` for only intended files and record pre-existing unrelated failures without modifying them.
- [ ] **Step 4: Commit** any documentation evidence change, otherwise make no empty commit.
