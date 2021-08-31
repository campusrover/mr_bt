#!/usr/bin/env python3

import sys
sys.path.append("..") # Adds higher directory to python modules path.

import rospy
import numpy as np 
from sensor_msgs.msg import LaserScan

from ..nodes import Update


'''
This class adds a variable to the blackboard which specifies an angular velocity for which
to turn so that the robot can track an object.
'''
class AngularPID(Update):


    def __init__(self, pid_err_var_name, nearest_dist_var_name, nearest_angle_var_name, kp, kd, kp2, dwall, offset=0):

        self.pid_err_var_name = pid_err_var_name
        self.nearest_dist_var_name = nearest_dist_var_name
        self.nearest_angle_var_name = nearest_angle_var_name

        self.kp = kp
        self.kd = kd
        self.kp2 = kp2

        self.dwall = dwall
        self.offset = offset

        self.dmin_prev = None
        self.tn_prev = rospy.Time.now().to_sec()


    def tick(self, blackboard):

        try:

            tn = rospy.Time.now().to_sec()
            amin = blackboard[self.nearest_angle_var_name]

            dmin = blackboard[self.nearest_dist_var_name]
            if self.dmin_prev is None:
                self.dmin_prev = dmin
                
            wall_err = dmin - self.dwall
            wall_err_prev = self.dmin_prev - self.dwall

            PDct = self.kp * wall_err + self.kd * ( (wall_err - wall_err_prev) / (tn - self.tn_prev + 1e-10) )

            if amin >= 3.1415:
                di = -1
            else:
                di = 1

            a_err = amin - (self.offset)*di

            Pct = self.kp2*a_err

            final_angular_velocity = (PDct + Pct)
            
            blackboard[self.pid_err_var_name] = final_angular_velocity*di

            self.tn_prev = tn
            self.dmin_prev = dmin

            return "success"

        except:

            return "failure"


class LinearPID(Update):

    def __init__(self, linear_pid_var_name, diff_var_name, max_vel, offset=0):

        self.linear_pid_var_name = linear_pid_var_name
        self.diff_var_name = diff_var_name
        self.max_vel = max_vel
        self.offset = offset



    def tick(self, blackboard):

        try:

            diff = blackboard[self.diff_var_name] - self.offset

            x = 10 * diff

            y = self.max_vel * np.tanh(x)

            blackboard[self.linear_pid_var_name] = y

            return "success"
        
        except:

            return "failure"





