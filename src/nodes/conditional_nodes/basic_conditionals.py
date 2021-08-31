#!/usr/bin/env python3

import sys
sys.path.append("..") # Adds higher directory to python modules path.

import rospy 
import numpy as np 

from ..nodes import Conditional



class BoolVar(Conditional):


    def __init__(self, var_name):

        self.var_name = var_name


    def condition(self, blackboard):

        return blackboard[self.var_name]


class BoolVarNot(Conditional):


    def __init__(self, var_name):

        self.var_name = var_name


    def condition(self, blackboard):

        return not blackboard[self.var_name]