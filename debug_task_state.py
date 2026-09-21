"""跟踪任务态执行，捕获并诊断死循环。"""

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from navigation.simulation.composition import build_simulation_runner


def fmt_location(location):
    if location is None:
        return "None"
    if hasattr(location, "node_id"):
        return "AtNode({})".format(location.node_id)
    if hasattr(location, "traversal_id"):
        return "OnCruiseEdge({}, {:.0f}mm)".format(
            location.traversal_id, getattr(location, "progress_mm", 0)
        )
    return repr(location)


def fmt_pose(pose):
    if pose is None:
        return "None"
    return "({:.1f},{:.1f},{:.1f})".format(pose.x_mm, pose.y_mm, pose.yaw_deg)


def get_nav(runner):
    if runner.navigation_runtime is None:
        return None
    try:
        return runner.navigation_runtime.snapshot()
    except Exception:
        return None


def get_attr(obj, name, default=None):
    return getattr(obj, name, default) if obj is not None else default


def main(max_steps=200):
    runner = build_simulation_runner(seed=0)
    runner.start()

    last_diag_count = 0
    last_state = None
    last_plan = None
    last_diag_len = 0
    loop_streak = 0

    for step in range(max_steps):
        world = runner.world.snapshot()
        nav = get_nav(runner)
        logical = runner.state_store.robot_state() if runner.state_store else None

        nav_state = get_attr(nav, "state")
        nav_substate = get_attr(nav, "substate")
        active_plan = get_attr(nav, "active_choreography")
        current_req = get_attr(nav, "current_request")
        plan_id = get_attr(active_plan, "choreography_id")
        diag = get_attr(nav, "diagnostics", []) or []

        # 死循环检测：状态和剧本都没变，诊断持续增长
        if nav_state == last_state and plan_id == last_plan and len(diag) > last_diag_len:
            loop_streak += 1
        else:
            loop_streak = 0
        last_state = nav_state
        last_plan = plan_id
        last_diag_len = len(diag)

        print("=" * 78)
        print("STEP {:3d}  loop_streak={}".format(step, loop_streak))
        print("  world_pose  = {}".format(fmt_pose(world.pose)))
        print("  logical     = {} hdg={}".format(
            fmt_location(logical.location if logical else None),
            logical.heading_deg if logical else None,
        ))
        print("  nav_state   = {}".format(nav_state))
        print("  substate    = {}".format(nav_substate))
        print("  active_plan = {}".format(plan_id))
        print("  current_req = {}".format("yes" if current_req is not None else "no"))

        if len(diag) > last_diag_count:
            print("  --- new diagnostics ---")
            for line in diag[last_diag_count:]:
                print("    {}".format(line))
            last_diag_count = len(diag)

        if loop_streak >= 20:
            print()
            print(">>> 检测到死循环：state 和 plan 连续 20 步未变，但诊断持续增长")
            print(">>> 最后 30 条诊断：")
            for line in diag[-30:]:
                print("    {}".format(line))
            return

        if get_attr(nav, "mission_finished", False):
            print()
            print(">>> MISSION FINISHED")
            return

        advanced = runner.step()
        if not advanced:
            print("  >>> step 返回 False：没有待完成请求")

    print()
    print("=" * 78)
    print("最终诊断（最后 50 条）")
    print("=" * 78)
    if nav:
        for line in (get_attr(nav, "diagnostics", []) or [])[-50:]:
            print(line)


if __name__ == "__main__":
    main()