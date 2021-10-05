#!/usr/bin/env python3

from .node import Node



class ParentNode(Node):
    '''
    This class is a blueprint for different types of parent nodes in the behavior tree. 
    All parents will take in a list of child nodes as a parameter when initialized.
    The child nodes can either be action/conditional nodes, sequencers, or other selectors.
    '''

    def __init__(self, children):

        super().__init__()

        self.num_children = len(children)
        
        self.children = children