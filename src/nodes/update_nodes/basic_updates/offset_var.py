#!/usr/bin/env python3

import rospy
import numpy as np
import time

from ...nodes import Update




class OffsetVar(Update):


    def __init__(self, var_name, offset_var_name, offset):

        self.var_name = var_name

        self.offset_var_name = offset_var_name

        self.offset = offset


    def tick(self, blackboard):

        try:

            blackboard[self.offset_var_name] = blackboard[self.var_name] + self.offset

            return "success"

        except:

            return "failure"
        