"""定义任务生命周期的唯一受控注册表。"""

# 导入不可变数据类和替换工具，任务状态变更必须创建新快照。
from dataclasses import dataclass, replace
# 导入 Python 3.8 兼容的类型注解工具。
from typing import Dict, Iterable, Optional, Tuple

# 导入长期任务及其生命周期，注册表不解释路线和动作。
from .tasks import Task, TaskLifecycle


@dataclass(frozen=True)
class TaskTransition:
    """一次任务生命周期操作的可观察结果。

    谁调用：`Coordinator` 提交、确认或取消任务后读取。
    谁响应：`TaskRegistry` 返回本对象，说明操作是否被接受以及前后状态。
    输入输出：不接收额外输入；输出含任务快照和可选拒绝原因。
    状态影响：本对象不改状态；`accepted=True` 代表注册表已经更新。
    """

    # 本次操作是否满足生命周期规则并已经写入注册表。
    accepted: bool
    # 操作前任务；未知任务时为空。
    before: Optional[Task]
    # 操作后任务；拒绝未知任务时为空，其他拒绝时保持原任务。
    after: Optional[Task]
    # 拒绝时的业务原因；成功操作时为空。
    reason: Optional[str] = None


class TaskRegistry:
    """保存任务生命周期的最小可变所有者，不承担路线选择职责。

    谁调用：后续 `Coordinator` 在任务动作提交、成功确认或计划取消时调用。
    谁响应：本类返回任务只读快照或 `TaskTransition`。
    输入输出：构造时输入任务序列；查询返回元组，状态操作输入任务标识并返回变更结果。
    状态影响：只有合法开始、完成或取消会替换对应任务的生命周期。
    """

    def __init__(self, tasks: Iterable[Task]) -> None:
        """以一组初始任务创建独立注册表，并保持输入顺序作为读取顺序。"""

        # 将输入立即冻结，避免外部可变列表或生成器改变注册表初始顺序。
        initial_tasks = tuple(tasks)
        # 保存稳定任务标识顺序，供面板和测试获得可复现的只读视图。
        self._task_ids: Tuple[str, ...] = tuple(task.task_id for task in initial_tasks)
        # 重复标识会让异步任务中断无法唯一匹配，必须在装配阶段显式拒绝。
        if len(set(self._task_ids)) != len(self._task_ids):
            raise ValueError("任务标识不能重复")
        # 按稳定标识保存当前任务；任务本身不可变，状态变更将替换该值。
        self._tasks_by_id: Dict[str, Task] = {task.task_id: task for task in initial_tasks}

    def list_tasks(self) -> Tuple[Task, ...]:
        """按初始顺序返回全部任务的当前不可变快照。

        谁调用：`Coordinator`、Dashboard 和测试。
        谁响应：本类根据稳定任务顺序返回当前任务值。
        输入输出：不接收参数；输出任务元组，不暴露内部字典。
        状态影响：只读查询，不改变任何任务生命周期。
        """

        # 按保存顺序读取，避免字典实现细节成为业务排序规则。
        return tuple(self._tasks_by_id[task_id] for task_id in self._task_ids)

    def pending_tasks(self) -> Tuple[Task, ...]:
        """返回当前仍可被目标派生器消费的待办任务。

        谁调用：后续 `Coordinator` 创建正常规划候选时调用。
        谁响应：本类从全部任务中过滤生命周期为 `PENDING` 的快照。
        输入输出：不接收参数；输出待办任务元组。
        状态影响：只读查询，不会自动开始、取消或完成任务。
        """

        # 只保留尚未提交给任务系统的任务，其他状态不能重复规划。
        return tuple(task for task in self.list_tasks() if task.lifecycle is TaskLifecycle.PENDING)

    def register(self, task: Task) -> TaskTransition:
        """登记一个新的待办任务，供协调器在发现运行时任务后调用。"""

        if not isinstance(task, Task):
            raise TypeError("task 必须是 Task")
        if task.task_id in self._tasks_by_id:
            return TaskTransition(False, self._tasks_by_id[task.task_id], self._tasks_by_id[task.task_id], "任务标识已存在")
        if any(
            existing.kind is task.kind and existing.target_id == task.target_id
            for existing in self._tasks_by_id.values()
        ):
            return TaskTransition(False, None, None, "任务目标已登记")
        self._task_ids = self._task_ids + (task.task_id,)
        self._tasks_by_id[task.task_id] = task
        return TaskTransition(True, None, task)

    def begin(self, task_id: str) -> TaskTransition:
        """将待办任务推进为执行中，防止同一任务重复提交。

        谁调用：后续 `Coordinator` 在任务动作真正提交给执行器前调用。
        谁响应：本类校验任务存在且为待办状态，返回变更结果。
        输入输出：输入任务标识；输出包含前后状态的 `TaskTransition`。
        状态影响：仅 `PENDING` 任务会被替换为 `EXECUTING`。
        """

        # 复用统一转换逻辑，要求当前状态必须是待办状态。
        return self._transition(task_id, TaskLifecycle.PENDING, TaskLifecycle.EXECUTING, "任务不是待办状态")

    def complete(self, task_id: str) -> TaskTransition:
        """将已执行任务推进为完成，调用者必须先完成外部结果校验。

        谁调用：后续 `Coordinator` 校验匹配成功中断和地图事实后调用。
        谁响应：本类校验任务正处于执行中，返回变更结果。
        输入输出：输入任务标识；输出包含前后状态的 `TaskTransition`。
        状态影响：仅 `EXECUTING` 任务会被替换为 `COMPLETED`。
        """

        # 注册表只在协调器已校验的前提下执行受控状态替换。
        return self._transition(task_id, TaskLifecycle.EXECUTING, TaskLifecycle.COMPLETED, "任务不是执行中状态")

    def cancel(self, task_id: str) -> TaskTransition:
        """将执行中任务退回待办，供恢复或重规划后重新选择。

        谁调用：后续 `Coordinator` 取消尚未完成的任务计划时调用。
        谁响应：本类校验任务正处于执行中，返回变更结果。
        输入输出：输入任务标识；输出包含前后状态的 `TaskTransition`。
        状态影响：仅 `EXECUTING` 任务会被替换为 `PENDING`，已完成任务不会回退。
        """

        # 仅允许撤销已占用但尚未确认完成的任务，避免完成事实与任务状态矛盾。
        return self._transition(task_id, TaskLifecycle.EXECUTING, TaskLifecycle.PENDING, "任务不是执行中状态")

    def _transition(self, task_id: str, expected: TaskLifecycle, target: TaskLifecycle, reason: str) -> TaskTransition:
        """在当前生命周期符合预期时替换任务值，否则返回拒绝结果。"""

        # 未知标识无法匹配任何异步动作，返回空快照而不是创建新任务。
        if task_id not in self._tasks_by_id:
            return TaskTransition(False, None, None, "未知任务")
        # 取得当前不可变任务快照，后续判断都基于这一份状态。
        before = self._tasks_by_id[task_id]
        # 非法转换必须保持原任务不变，并向协调器明确返回拒绝原因。
        if before.lifecycle is not expected:
            return TaskTransition(False, before, before, reason)
        # 用数据类替换创建目标生命周期的新任务，避免原地修改跨模块快照。
        after = replace(before, lifecycle=target)
        # 将新快照写回唯一任务所有者，后续查询才会看到状态变更。
        self._tasks_by_id[task_id] = after
        # 返回成功结果，供协调器记录事件和继续后续流程。
        return TaskTransition(True, before, after)
