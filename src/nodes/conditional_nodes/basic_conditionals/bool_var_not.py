#!/usr/bin/env python3
import rospy 
import numpy as np 

from ...nodes.conditional import Conditional



class BoolVarNot(Conditional):


    def __init__(self, var_name):

        self.var_name = var_name


    def condition(self, blackboard):

        return not blackboard[self.var_name]