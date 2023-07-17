from math import exp
from ...nodes.update import Update


class PathCorrection(Update):
    def __init__(
        self,
        correction_var_name: str,
        goal_value_var_name: str,
        current_value_var_name: str,
        max_vel: float,
        steepness: float = 5.0,
        offset: float = 0,
    ):
        super().__init__()

        self.correction = correction_var_name
        self.goal = goal_value_var_name
        self.curr = current_value_var_name

        self.correct = lambda goal_minus_actual: (
            #           (2 / (1 + math.e ** (-1 * steepness * (x - offset)))) - 1
            self.correct_with_sigmoid(goal_minus_actual, offset, max_vel, steepness)
        )

    def correct_with_sigmoid(self, goal_minus_actual, offset, maxvel, steepness):
        raw_sigmoid = 1 / (1 + exp(-steepness * (goal_minus_actual - offset))) - 0.5
        result = maxvel * 2 * raw_sigmoid
        print(
            f"******* result: {result:1.4f}, delta: {goal_minus_actual:1.2f}, maxvel:{maxvel:1.2f}, steep: {steepness:1.2f}, offset: {offset:1.2f}"
        )
        return result

    def update_blackboard(self, blackboard: dict):
        try:
            cur_val = blackboard[self.curr]
            goal_val = blackboard[self.goal]
            # print(f"goal: {goal_val:1.2f}    cur:{cur_val:1.2f}")
            correct_val = self.correct(goal_val)
            blackboard[self.correction] = correct_val
            return "success"
        except Exception as exception:
            print(f"Caught Exception: {exception}")
            return "failure"
