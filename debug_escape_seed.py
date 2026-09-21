"""seed=65 脱困问题专用调试脚本（重写版）。

关键改动：
- 每步打印 state / world_pose / robot_pose / logical / action_id，观察动作也能看到推进；
- 卡住检测改为「同一个 request_id 连续 N 步未被消费」，而不是「位置没变」；
- 真正的终止条件：「无 pending 且无在途请求」。
"""

import sys
import traceback
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from navigation.simulation.composition import build_simulation_runner


# ---------- 格式化工具 ----------

def fmt_pose(pose):
    if pose is None:
        return "None"
    return "({:.1f},{:.1f},{:.1f})".format(pose.x_mm, pose.y_mm, pose.yaw_deg)


def fmt_loc(loc):
    if loc is None:
        return "None"
    if hasattr(loc, "node_id"):
        entry = getattr(loc, "entry_traversal_id", None)
        if entry:
            return "AtNode({}, {})".format(loc.node_id, entry)
        return "AtNode({})".format(loc.node_id)
    if hasattr(loc, "traversal_id"):
        return "Edge({}, {:.0f}mm)".format(loc.traversal_id, getattr(loc, "progress_mm", 0))
    return repr(loc)


def get_attr(obj, name, default=None):
    return getattr(obj, name, default) if obj is not None else default


def diff_pose(a, b):
    if a is None or b is None:
        return None
    return (a.x_mm - b.x_mm, a.y_mm - b.y_mm, a.yaw_deg - b.yaw_deg)


def fmt_diff(d):
    if d is None:
        return "--"
    return "({:+.1f},{:+.1f},{:+.1f})".format(*d)


# ---------- ESCAPE 上下文 dump ----------

def dump_escape_context(runner, logical):
    print()
    print("  ===== ESCAPE 上下文 dump =====")

    map_snap = runner.state_store.runtime_map_snapshot() if runner.state_store else None
    if map_snap is not None:
        print("  blocked_edges   = {}".format(sorted(map_snap.blocked_edge_ids)))
        print("  clear_edges     = {}".format(sorted(map_snap.confirmed_clear_edge_ids)))
        print("  culvert_edges   = {}".format(sorted(map_snap.discovered_culvert_edge_ids)))

    coordinator = (
        getattr(runner.navigation_runtime, "_coordinator", None)
        or getattr(runner.navigation_runtime, "coordinator", None)
    )
    if coordinator is None:
        print("  !! 无法获取 coordinator")
        return

    tr = getattr(coordinator, "_task_registry", None)
    if tr is not None:
        print("  pending_tasks:")
        for task in tr.pending_tasks():
            print("    - {} ({}, target={})".format(
                task.task_id, task.kind.value, task.target_id
            ))
        print("  all_tasks:")
        for task in tr.list_tasks():
            print("    - {} ({}, target={}, lifecycle={})".format(
                task.task_id, task.kind.value, task.target_id, task.lifecycle.value
            ))
    print("  last_completed_turn = {}".format(getattr(coordinator, "last_completed_turn", None)))
    print("  task_requirements_met = {}".format(getattr(coordinator, "task_requirements_met", None)))

    planner = getattr(coordinator, "_route_planner", None)
    if planner is not None:
        try:
            assessment = planner.assess_escape()
            print("  EscapeAssessment:")
            for name in ("forward", "left", "right", "backward"):
                d = getattr(assessment, name, None)
                print("    {}: status={} worth_trying={} traversal={}".format(
                    name,
                    getattr(getattr(d, "status", None), "name", None),
                    getattr(d, "worth_trying", None),
                    getattr(d, "traversal_id", None),
                ))
        except Exception as exc:
            print("  assess_escape 失败：{}".format(exc))
            traceback.print_exc()

    topo = getattr(coordinator, "_topology", None)
    if topo is not None and logical is not None:
        loc = logical.location
        node_id = getattr(loc, "node_id", None)
        if node_id is not None:
            print("  outgoing_edges from {}:".format(node_id))
            for edge in topo.outgoing_cruise_edges(node_id):
                print("    {} (len={:.0f}mm, kind={})".format(
                    edge.traversal_id, edge.length_mm, edge.road_kind
                ))

    print("  ===== dump 结束 =====")
    print()


# ---------- 主流程 ----------

def main(seed=0, max_steps=300, stuck_threshold=5):
    runner = build_simulation_runner(seed=seed)
    runner.start()

    last_diag_len = 0
    escape_context_dumped = False

    last_stuck_request_id = None
    stuck_count = 0

    for step in range(max_steps):
        world = runner.world.snapshot()
        nav = runner.navigation_runtime.snapshot() if runner.navigation_runtime else None
        logical = runner.state_store.robot_state() if runner.state_store else None
        diag = getattr(nav, "diagnostics", []) or []
        nav_state = str(getattr(nav, "state", None))
        active_plan = get_attr(get_attr(nav, "active_choreography"), "choreography_id")
        current_req = getattr(nav, "current_request", None)
        action_id = getattr(current_req, "action_id", None) if current_req else None
        request_id = getattr(current_req, "request_id", None) if current_req else None

        # ---- 每步打印核心状态 ----
        print("STEP {:3d} state={:<15} world={:<22} robot={:<22} loc={:<40} action={}".format(
            step,
            nav_state,
            fmt_pose(world.pose),
            fmt_pose(logical.world_pose) if logical else "None",
            fmt_loc(logical.location if logical else None),
            action_id or "-",
        ))
        d = diff_pose(logical.world_pose, world.pose) if logical else None
        print("        plan={} diff={}".format(active_plan or "-", fmt_diff(d)))

        # ---- 新增诊断 ----
        if len(diag) > last_diag_len:
            for line in diag[last_diag_len:]:
                print("        | " + line)
            last_diag_len = len(diag)

        # ---- 首次进 ESCAPE，dump 上下文 ----
        if "escape" in nav_state.lower() and not escape_context_dumped:
            escape_context_dumped = True
            dump_escape_context(runner, logical)

        # ---- 卡住检测：同一个 request_id 连续 N 步未被消费 ----
        if request_id is not None and request_id == last_stuck_request_id:
            stuck_count += 1
        else:
            stuck_count = 0
            last_stuck_request_id = request_id

        if stuck_count >= stuck_threshold:
            print()
            print(">>> 检测到卡住：request_id={} 连续 {} 步未被消费".format(
                request_id, stuck_count
            ))
            dump_escape_context(runner, logical)
            break

        # ---- 推进一步 ----
        try:
            advanced = runner.step()
        except Exception:
            print()
            print(">>> runner.step() 抛异常：")
            traceback.print_exc()
            break

        # ---- 真正终止条件：无 pending 且无在途请求 ----
        if not advanced and current_req is None:
            print()
            print(">>> 无 pending 且无在途请求，退出")
            break

    print()
    print("=== 结束 ===")
    print("总步数：{}".format(step + 1))
    print("final state: {}".format(nav_state))
    print("active_plan: {}".format(active_plan))


if __name__ == "__main__":
    # 两个常用 seed：0 跑正常流程，65 跑脱困场景
    main(seed=0, max_steps=300)