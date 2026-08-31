from dataclasses import dataclass, field


@dataclass(frozen=True)
class TaskResult:
    success: bool
    final_pose: object = None
    map_updates: tuple[object, ...] = ()
    failure_reason: str = ""


class TaskExecution:
    def __init__(self, mission_state):
        self.mission_state = mission_state

    def begin(self, task_id: str) -> bool:
        return self.mission_state.begin(task_id)

    def finish(self, task_id: str, result: TaskResult, now: float = 0.0,
               cooldown_s: float = 0.0) -> TaskResult:
        if result.success:
            self.mission_state.succeed(task_id)
        else:
            self.mission_state.fail(task_id, now, cooldown_s)
        return result
