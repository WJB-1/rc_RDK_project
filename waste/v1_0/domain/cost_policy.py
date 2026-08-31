from dataclasses import dataclass


@dataclass(frozen=True)
class CostPolicy:
    distance_weight: float = 1.0
    turn_weight: float = 0.0
    risk_weight: float = 0.0
    retry_penalty: float = 0.0

    def __post_init__(self):
        if min(self.distance_weight, self.turn_weight, self.risk_weight, self.retry_penalty) < 0:
            raise ValueError("cost weights must be non-negative")

    @classmethod
    def explore(cls) -> "CostPolicy":
        return cls(distance_weight=1.0, turn_weight=1.0, risk_weight=1.0)

    def route_cost(self, distance_mm: float, turn_cost: float = 0.0, risk: float = 0.0) -> float:
        return (max(0.0, distance_mm) * self.distance_weight
                + max(0.0, turn_cost) * self.turn_weight
                + max(0.0, risk) * self.risk_weight)
