#!/usr/bin/env python3

from math import atan2, pi
from ...nodes.update import Update

# from utils.py

def turn_to_target(yaw, x0, y0, x1, y1):
    """ Current position is x0, y0. Current orientation is angle yaw, where 0 is North and positive angles go counterclockwise. 
    Target destination is x1, y1. Compute needed turn to point directly at x1, y1."""
    delta_x = x1 - x0
    delta_y = y1 - y0
# Calculate the angle between the current orientation and the target point using the arctangent function: θ = atan2(Δy, Δx).
    angle_to_target = atan2(delta_y, delta_x)
# Calculate the angle difference between the target angle and the current angle: Δθ = θ - A.
    turn_amount = angle_to_target - yaw
    return turn_amount

def normalize_angle(angle: float):
    """Convert an angle to a normalized angle. A normalized angle, for us, is
    in radians, -pi < angle < pi. 0 is at 3 o'clock, positive is counterclockwise"""
    angle_out = angle
    if angle_out < 0:
        while abs(angle_out) > pi:
            angle_out += 2*pi
    elif angle_out > 0:
        while angle_out > pi:
            angle_out = -2 * pi + angle_out
    return angle_out



class AngleToPosition(Update):

    def __init__(self, goal_position_var_name, curr_position_var_name, goal_rotation_var_name, rotation_var_name):

        super().__init__()

        self.goal_position_var_name = goal_position_var_name
        self.curr_position_var_name = curr_position_var_name
        self.goal_rotation_var_name = goal_rotation_var_name
        self.rotation_var_name = rotation_var_name


    def update_blackboard(self, blackboard:dict) -> str:

        goal_pos = blackboard[self.goal_position_var_name]
        curr_pos = blackboard[self.curr_position_var_name]
        rot = blackboard[self.rotation_var_name]

        # curr_vect = [np.cos(rot), np.sin(rot)]
        # goal_vect = [goal_pos[0]-(0-curr_pos[0]), goal_pos[1]-(0-curr_pos[1])]
        # goal_angle = np.arctan(goal_vect[1]/goal_vect[0])
        # if goal_vect[1] < 0:
        #     goal_angle = np.pi + goal_angle
        goal_angle = turn_to_target(
            rot,
            curr_pos[0],
            curr_pos[1],
            goal_pos[0],
            goal_pos[1],
        )
        blackboard["temp"] = goal_angle
        blackboard[self.goal_rotation_var_name] = normalize_angle(goal_angle)
        return 'success'

