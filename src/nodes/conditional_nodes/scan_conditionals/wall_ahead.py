#!/usr/bin/env python3

import rospy 
import numpy as np 

from ...nodes import Conditional


'''
Returns "success" if there is a wall ahead of the robot according to the LIDAR scanner, or
otherwise returns "failure".
'''
class WallAhead(Conditional):

    def __init__(self, dist, fov):

        self.dist = dist

        self.view_frac = fov/720


    def condition(self, blackboard):

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

        return wall_ahead