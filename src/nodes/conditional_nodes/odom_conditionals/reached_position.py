#!/usr/bin/env python3


import rospy 
import numpy as np 

from ...nodes import Conditional


from nav_msgs.msg import Odometry



class ReachedPosition(Conditional):

    def __init__(self, odom_var_name, goal_pos_var_name, error):

        self.odom_var_name = odom_var_name
        self.goal_pos_var_name = goal_pos_var_name
        self.error = error

    
    def condition(self, blackboard):


        pos = [blackboard[self.odom_var_name].pose.pose.position.x, blackboard[self.odom_var_name].pose.pose.position.y]
        goal = blackboard[self.goal_pos_var_name]
        dist = np.sqrt((goal[0]-pos[0])**2 + (goal[1]-pos[1])**2)

        return dist < self.error