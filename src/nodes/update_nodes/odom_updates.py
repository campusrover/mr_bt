#!/usr/bin/env python3

import sys
sys.path.append("..") # Adds higher directory to python modules path.

import rospy
import numpy as np 
import math
from nav_msgs.msg import Odometry

from ..nodes import Update


class GetPosition(Update):

    def __init__(self, odom_var_name, position_var_name):

        self.odom_var_name = odom_var_name
        self.position_var_name = position_var_name


    def tick(self, blackboard):

        try:

            pose = [blackboard[self.odom_var_name].pose.pose.position.x, blackboard[self.odom_var_name].pose.pose.position.y]

            blackboard[self.position_var_name] = pose

            return "success"

        except:

            return "failure"





class GetRotation(Update):


    def __init__(self, odom_var_name, rotation_var_name, degrees=True):

        self.odom_var_name = odom_var_name
        self.rotation_var_name = rotation_var_name

        if degrees:
            # Conversion between radians and degrees
            self.mult = 180/3.1415
        else:
            self.mult = 1


    def tick(self, blackboard):

        try:

            # Getting orientation from msg
            ori = blackboard[self.odom_var_name].pose.pose.orientation
            x = ori.x
            y = ori.y
            z = ori.z
            w = ori.w

            # Converting the quaternion values to radians, and then to degrees if the user specifies
            t3 = 2.0 * (w * z + x * y)
            t4 = 1.0 - 2.0 * (y * y + z * z)
            rot = math.atan2(t3, t4)

            if rot < 0:
                rot += 2*3.1415

            blackboard[self.rotation_var_name] = rot * self.mult

            return "success"

        except:

            return "failure"



class AngleToPosition(Update):

    def __init__(self, goal_position_var_name, curr_position_var_name, goal_rotation_var_name, rotation_var_name):

        self.goal_position_var_name = goal_position_var_name
        self.curr_position_var_name = curr_position_var_name
        self.goal_rotation_var_name = goal_rotation_var_name
        self.rotation_var_name = rotation_var_name

    def tick(self, blackboard):

        goal_pos = blackboard[self.goal_position_var_name]
        curr_pos = blackboard[self.curr_position_var_name]
        rot = blackboard[self.rotation_var_name]

        curr_vect = [np.cos(rot), np.sin(rot)]

        goal_vect = [goal_pos[0]-(0-curr_pos[0]), goal_pos[1]-(0-curr_pos[1])]

        goal_angle = np.arctan(goal_vect[1]/goal_vect[0])

        goal_rotation = (goal_angle-rot)*(180/3.1415) 

        if goal_rotation < -180:

            goal_rotation += 360

        elif goal_rotation > 180:

            goal_rotation -= 360

        blackboard[self.goal_rotation_var_name] = goal_rotation * (3.1415/180)

        return 'success'



class DistToPosition(Update):

    def __init__(self, goal_position_var_name, curr_position_var_name, dist_var_name):

        self.goal_position_var_name = goal_position_var_name

        self.curr_position_var_name = curr_position_var_name

        self.dist_var_name = dist_var_name


    def tick(self, blackboard):


        curr_pos = blackboard[self.curr_position_var_name]

        goal_pos = blackboard[self.goal_position_var_name]

        dist = np.sqrt((goal_pos[0]-curr_pos[0])**2 + (goal_pos[1]-curr_pos[1])**2)

        blackboard[self.dist_var_name] = dist

        return "success"