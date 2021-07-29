#!/usr/bin/env python3

import sys
sys.path.append("..") # Adds higher directory to python modules path.

import rospy 
import numpy as np 

from nodes import Conditional

from nav_msgs.msg import Odometry



class ReachedPosition(Conditional):

    def __init__(self, odom_var_name, goal_pos_var_name, error):

        self.odom_var_name = odom_var_name
        self.goal_pos_var_name = goal_pos_var_name
        self.error = error

    
    def condition(self, blackboard):


        pose = [blackboard[self.odom_var_name].pose.pose.position.x, blackboard[self.odom_var_name].pose.pose.position.y]
        goal = blackboard[self.goal_pos_var_name]

        return (abs(pose[0]-goal[0]) < self.error) and (abs(pose[1]-goal[1]) < self.error)