#!/usr/bin/env python3

from .node import Node

class Update(Node):
    '''
    The Update class is a leaf node in the behavior tree which performs some calculation/algorithm
    on information in the blackboard and updates the blackboard with new information.

    Each update should be tick based, so during each tick the .tick() method of an update
    will either return "failure", or "success" depending whether or not the update is successful

    This class is not meant to be initialized, but serves as an abstract parent class for users
    to construct their own updates with custom methods.
    '''

