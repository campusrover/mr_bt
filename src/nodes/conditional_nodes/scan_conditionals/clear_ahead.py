#!/usr/bin/env python3

from typing import Union
import rospy 
import numpy as np 
from ...nodes.conditional import Conditional


'''
Inverse of WallAhead conditional.
'''
class ClearAhead(Conditional):


    def __init__(self, dist: float, fov: Union[float, int]):

        super(ClearAhead, self).__init__()

        self.dist = dist
        self.view_frac = fov/360

    def condition(self, blackboard: dict) -> bool:
        ranges = np.array(blackboard['/scan'].ranges)
        ranges[ranges == 0] = 999
        n = ranges.size 
        wall_to_left = np.min(ranges[0:int(n*self.view_frac)]) <= self.dist
        wall_to_right = np.min(ranges[n-int(n*self.view_frac):]) <= self.dist
        wall_ahead = wall_to_left or wall_to_right

        if wall_ahead:
            if wall_to_left:
                print("Wall to the left!")
            else:
                print("Wall to the right!")
        else:
            print('No wall ahead')

        return not wall_ahead
