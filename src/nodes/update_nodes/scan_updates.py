#!/usr/bin/env python3

import sys
sys.path.append("..") # Adds higher directory to python modules path.

import rospy
import numpy as np 
from sensor_msgs.msg import LaserScan

from ..nodes import Update


'''
Calculates the angle (in degrees) of the nearest wall to the robot according to the
LIDAR scanner.
'''
class CalcNearestWallAngle(Update):


    def __init__(self, scan_var_name, angle_var_name, degrees=False):

        self.angle_var_name = angle_var_name

        self.scan_var_name = scan_var_name

        self.degrees = degrees

    
    def tick(self, blackboard):

        try:

            ranges = np.array(blackboard[self.scan_var_name].ranges)

            ranges[ranges <= 0.07] = 999

            min_ind = np.argmin(ranges)

            ratio = 360/ranges.size

            min_angle = min_ind*ratio

            if self.degrees:

                blackboard[self.angle_var_name] = min_angle
            
            else:

                blackboard[self.angle_var_name] = min_angle * (3.1415/180)

            return "success"

        except:

            return "failure"


'''
Calculates the closest distance to the robot according to the
LIDAR scanner.
'''
class CalcNearestDist(Update):


    def __init__(self, scan_var_name, dist_var_name):

        self.dist_var_name = dist_var_name

        self.scan_var_name = scan_var_name

    
    def tick(self, blackboard):

        try:

            ranges = np.array(blackboard[self.scan_var_name].ranges)

            ranges[ranges <= 0.07] = 999

            min_dist = np.min(ranges)

            blackboard[self.dist_var_name] = min_dist

            return "success"

        except:

            return "failure"


class CalcAvgFrontDist(Update):


    def __init__(self, scan_var_name, dist_var_name, fov):

        self.scan_var_name = scan_var_name

        self.dist_var_name = dist_var_name

        self.view_frac = fov/720


    def tick(self, blackboard):

        try:

            ranges = np.array(blackboard['/scan'].ranges)

            ranges[ranges <= 0.1] = 0

            n = ranges.size 

            wall_to_left = ranges[0:int(n*self.view_frac)]
            wall_to_right = ranges[n-int(n*self.view_frac):]

            total_avg = (np.average(wall_to_left) + np.average(wall_to_right))/2

            blackboard[self.dist_var_name] = total_avg

            return "success"

        except:

            return "failure"


# class DistAtAngle(Update):

#     def __init__(self, angle_var_name, scan_var_name):

#         self.angle_var_name = angle_var_name
#         self.scan_var_name = scan_var_name


#     def tick()



    