#!/usr/bin/env python3

from .parent_node import ParentNode

        

class Selector(ParentNode):
    '''
    The Selector class is a parent node in the behavior tree which ticks each of its children nodes
    in left-right order until one of them returns "success" or "running", and then returns the
    status back up the tree. If each child returns "failure", then the Selector will return 
    "failure" back up the tree.
    '''
        
    def tick(self, blackboard):

        status = 'failure'
        i = 0
        while (status == 'failure') and (i < self.num_children):

            status = self.children[i].tick(blackboard)
            i += 1
        
        return status