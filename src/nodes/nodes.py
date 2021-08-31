#!/usr/bin/env python3

import rospy

from abc import ABC, abstractmethod
import random
from concurrent.futures import ThreadPoolExecutor


class Node(ABC):
    '''
    The Node class is an abstract class for every type of node in the behavior tree.
    This class is not meant to be initialized and instead used as a blueprint for other types
    of nodes.
    '''

    def __init__(self):

        return None 


    '''
    When tick() is called on a node it will return either "failure" if the action(s) associated with 
    the node have failed, "success" if they have completed, or "running" if they are still in progress.
    '''
    @abstractmethod
    def tick(self):

        options = ['failure', 'success', 'running']

        status = random.choice(options)

        return status


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


class Multitasker(ParentNode):
    '''
    The Multitasker class is a parent node in the behavior tree which utilizes multithreading in Python
    to simultaneously tick each of it's children nodes at the same time. Each of it's children nodes will
    run in their own threads and the results are only gathered once all of the children have returned a
    status. If one or more of the children nodes returns the "failure" status, then the Multitasker will return
    "failure". Otherwise, if one or more of the children nodes returns "running", then the Multitasker will
    return "running". If all of the children nodes return "success", then the Multitasker will also return "success".
    '''

    def tick(self, blackboard):

        statuses = []

        with ThreadPoolExecutor() as executor:

            threads = [executor.submit(child.tick, blackboard) for child in self.children]
            statuses = [thread.result() for thread in threads]

        if "failure" in statuses:

            return "failure"

        elif "running" in statuses:
            
            return "running"

        else:

            return "success"


class Action(Node):
    '''
    The Action class is a leaf node in the behavior tree which completes an action
    specified in the __init__ method. The user is required to customize their action 
    methods using this blueprint as a guide.

    Each action should be tick based, so during each tick the .tick() method of an action
    will either return "running", "failure", or "success" depending on the state of the action.

    This class is not meant to be initialized, but serves as an abstract parent class for users
    to construct their own actions with custom methods.
    '''


class Update(Node):
    '''
    The Update class is a leaf node in the behavior tree which performs some calculation/algorithm
    on information in the blackboard and updates the blackboard with new information.

    Each update should be tick based, so during each tick the .tick() method of an update
    will either return "failure", or "success" depending whether or not the update is successful

    This class is not meant to be initialized, but serves as an abstract parent class for users
    to construct their own updates with custom methods.
    '''


class Conditional(Node):
    '''
    The Conditional class is a leaf node in the behavior tree which returns either
    "success" or "failure" based on the boolean output from the condition function.
    Note that unlike other types of behavior tree nodes, a Conditional node will never
    return "running".

    The condition functon should be user defined and return a boolean value. 

    This class is not meant to be initialized, but serves as an abstract parent class for
    users to construct their own Conditional nodes with custom conditional functions.
    '''

    
    @abstractmethod
    def condition(self, blackboard):

        return True


    def tick(self, blackboard):

        condition_met = self.condition(blackboard)

        if condition_met:
            
            return 'success'
        
        else:

            return 'failure'       


class Test(Node):

    def __init__(self, print_statement):

        super().__init__()

        self.print_statement = print_statement

    
    def tick(self, blackboard):

        print(self.print_statement)

        return 'success'