#!/usr/bin/env python3

import rospy 
import numpy as np 

from ...nodes import Conditional



class WallAtAngleDynamic(Conditional):

    def __init__(self, angle_var_name, dist, fov):

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




class WallAtAngleDynamic(Conditional):

    def __init__(self, angle_var_name, scan_var_name, dist, fov):

        self.angle_var_name = angle_var_name
        self.scan_var_name = scan_var_name
        self.dist = dist
        self.fov = fov


    def condition(self, blackboard):

        ranges = blackboard[self.scan_var_name].ranges

        angle =  blackboard[self.angle_var_name]

        ranges[ranges == 0] = 999

        ratio = int(ranges.size/360)

        return ranges[ratio*angle] >= self.dist