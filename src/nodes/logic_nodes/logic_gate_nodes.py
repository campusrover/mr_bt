#!/usr/bin/env python3

import sys
sys.path.append("..") # Adds higher directory to python modules path.

from abc import ABC, abstractmethod
from ..nodes import ParentNode


class LogicGate(ParentNode):
    '''
    Logic nodes are a specialized type of parent node which takes in Conditional nodes
    as children and performs boolean logic on the outputs of its children. If there are more
    than 2 children, then it will return the result of the nested conditionals.
    '''

    def __init__(self, children):
        
        super().__init__(children)

        self.result = {'success':True, 'failure':False}

    @abstractmethod
    def logic(self, res1, res2):
        
        return


    def perform_logic(self, child_list, blackboard):

        if len(child_list) == 2:
            
            res1 = self.result[child_list[0].tick(blackboard)]
            res2 = self.result[child_list[1].tick(blackboard)]
            
            return self.logic(res1, res2)

        else:
            
            res1 = self.result[child_list[0].tick(blackboard)]
            res2 = self.perform_logic(child_list[1:], blackboard)

            return self.logic(res1, res2)


    def tick(self, blackboard):

        result = self.perform_logic(self.children, blackboard)

        if result:
            return 'success'
        else:
            return 'failure'


class And(LogicGate):

    def logic(self, res1, res2):

        return res1 and res2


class Or(LogicGate):

    def logic(self, res1, res2):

        return res1 or res2


class Xor(LogicGate):

    def logic(self, res1, res2):

        return res1 != res2


class Nand(LogicGate):

    def logic(self, res1, res2):

        return not (res1 and res2)


class Nor(LogicGate):

    def logic(self, res1, res2):

        return not (res1 or res2)


class Xnor(LogicGate):

    def logic(self, res1, res2):

        return res1 == res2


