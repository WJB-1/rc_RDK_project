class RecoveryExecution:
    def __init__(self, actuator):
        self.actuator = actuator

    def execute(self, plan):
        feedback = []
        for step in plan.steps:
            feedback.append(self.actuator.send(step))
        return feedback
