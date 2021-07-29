#!/usr/bin/env python3

import rospy
import message_filters




class ROSBehaviorTree:

    '''
    When initializing a tree, the "root" parameter should be the root node of a pre-structured
    behavior tree. The "blackboard_vars" parameter should be a python list containing other lists
    of size 2. The structure of the inner lists should be ["variable_name", message_type].
    
    If a blackboard variable is expected to be a ROS topic, then its variable_name should be the name of the topic starting
    with a "/". Ex: "/scan"

    If a blackboard variable is expected to be a ROS topic, then its message_type in "blackboard_vars" should 
    be the ROS message that is expected to be. Otherwise, you may leave the message type as either None or the 
    initial value that you want the variable to be.

    The full structure of the list should be as follows:

            [
                ["/some_ROS_topic", msg.ROSMessageType],
                ["some_integer_variable", None],
                ["some_boolean_variable", False]
            ]

    '''
    def __init__(self, root, blackboard, print_vars=[]):

        self.print_vars = print_vars
        
        self.curr_tick = 1

        self.root = root

        self.blackboard = blackboard

        subscribers = []
        self.topic_vars = []

        for var in blackboard:

            if var[0] == "/":

                subscribers.append(message_filters.Subscriber(var, blackboard[var]))
                self.topic_vars.append(var)

        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, 10, 0.1, allow_headerless=True)

        self.ts.registerCallback(self.cb)


    def cb(self, *args):

        for i, arg in enumerate(args):
            self.blackboard[self.topic_vars[i]] = arg

        status = self.root.tick(self.blackboard)

        print("\n\nTick {}: {}\n".format(self.curr_tick, status))
        for var in self.print_vars:
            print(var + ": " + str(self.blackboard[var]))

        self.curr_tick += 1
    
