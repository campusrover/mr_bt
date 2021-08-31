#!/usr/bin/env python3

import sys
sys.path.append("..") # Adds higher directory to python modules path.

from abc import ABC, abstractmethod
from ..nodes import ParentNode


class If(ParentNode):

    def __init__(self, children):

        super().__init__(children)

        if (self.num_children % 2) == 0:

            self.is_else = False

        else:

            self.is_else = True


    def tick(self, blackboard):

        i = 0

        while i < self.num_children-1:

            print(i)

            condition_met = self.children[i].tick(blackboard)

            if condition_met == 'success':

                result = self.children[i + 1].tick(blackboard)

                return result

            i += 2

        if self.is_else:

            result = self.children[-1].tick(blackboard)

            return result
            
        return 'failure'


