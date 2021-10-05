#!/usr/bin/env python3

from .parent_node import ParentNode

class Sequencer(ParentNode):
    '''
    The Sequencer class is a parent node in the behavior tree which ticks each of its children nodes
    in left-right order until one of them returns "failure" or "running", and then returns the
    status back up the tree. If each child returns "success", then the Sequencer will return 
    "success" back up the tree.
    '''

    def tick(self, blackboard):

        status = 'success'
        i = 0

        while (status == 'success') and (i < self.num_children):

            status = self.children[i].tick(blackboard)
            i += 1

        return status