#!/usr/bin/env python3

import rospy
import numpy as np 
import math
from nav_msgs.msg import Odometry

from ...nodes.update import Update


class GetPosition(Update):

    def __init__(self, odom_var_name, position_var_name):

        super().__init__()

        self.odom_var_name = odom_var_name
        self.position_var_name = position_var_name


    def update_blackboard(self, blackboard:dict) -> str:

        try:

            pose = [blackboard[self.odom_var_name].pose.pose.position.x, blackboard[self.odom_var_name].pose.pose.position.y]

            blackboard[self.position_var_name] = pose

            return "success"

        except:

            return "failure"

