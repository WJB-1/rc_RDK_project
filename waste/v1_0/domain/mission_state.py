from dataclasses import dataclass
from enum import Enum


class MissionStatus(str, Enum):
    PENDING = "pending"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    COOLDOWN = "cooldown"


@dataclass
class _MissionRecord:
    status: MissionStatus = MissionStatus.PENDING
    cooldown_until: float = 0.0
    attempts: int = 0


class MissionState:
    """Single-writer state for checkpoint and culvert tasks."""

    def __init__(self):
        self._records: dict[str, _MissionRecord] = {}

    def _record(self, task_id: str) -> _MissionRecord:
        return self._records.setdefault(task_id, _MissionRecord())

    def begin(self, task_id: str) -> bool:
        record = self._record(task_id)
        if record.status is MissionStatus.SUCCEEDED:
            return False
        record.status = MissionStatus.EXECUTING
        record.attempts += 1
        return True

    def succeed(self, task_id: str) -> None:
        self._record(task_id).status = MissionStatus.SUCCEEDED

    def fail(self, task_id: str, now: float, cooldown_s: float) -> None:
        record = self._record(task_id)
        record.status = MissionStatus.COOLDOWN
        record.cooldown_until = now + max(0.0, cooldown_s)

    def is_completed(self, task_id: str) -> bool:
        return self._record(task_id).status is MissionStatus.SUCCEEDED

    def is_candidate(self, task_id: str, now: float = 0.0) -> bool:
        record = self._record(task_id)
        if record.status is MissionStatus.SUCCEEDED:
            return False
        if record.status is MissionStatus.COOLDOWN and now < record.cooldown_until:
            return False
        if record.status is MissionStatus.COOLDOWN:
            record.status = MissionStatus.PENDING
        return True

    def status(self, task_id: str) -> MissionStatus:
        return self._record(task_id).status

    def attempts(self, task_id: str) -> int:
        return self._record(task_id).attempts
