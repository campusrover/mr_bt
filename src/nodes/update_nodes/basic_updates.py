#!/usr/bin/env python3
# | FlipBoolVar IncrementVar OffsetVar
import sys
sys.path.append("..") # Adds higher directory to python modules path.

import rospy
import numpy as np
import time

from ..nodes import Update


'''
Performs a "not" operation on a boolean variable in the blackboard.
'''
class FlipBoolVar(Update):


    def __init__(self, var_name):

        self.var_name = var_name


    def tick(self, blackboard):

        try:

            blackboard[self.var_name] = not blackboard[self.var_name]

            return "success"
        
        except:

            return "failure"


'''
Increments a number variable in the blackboard by an amount specified at initialization.
'''
class IncrementVar(Update):


    def __init__(self, var_name, increment):

        self.var_name = var_name
        self.increment = increment

    
    def tick(self, blackboard):

        try:

            blackboard[self.var_name] += self.increment

            return "success"

        except:

            return "failure"


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
        