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
    def __init__(self, root, blackboard, rate = 0.034, print_vars=[]):

        self.rate = rate

        self.print_vars = print_vars
        
        self.curr_tick = 1

        self.last_tick_time = rospy.get_time()

        self.root = root

        self.blackboard = blackboard

        subscribers = []

        for var in blackboard:

            # Creates a new subscriber for each topic specified in the blackboard
            if var[0] == "/":

                subscribers.append(rospy.Subscriber(var, blackboard[var], self.cb, var))

                self.blackboard[var] = None


    def tick_root(self):

        status = self.root.tick(self.blackboard)

        print("\n\nTick {}: {}\n".format(self.curr_tick, status))
        for var in self.print_vars:
            print(var + ": " + str(self.blackboard[var]))

        self.curr_tick += 1


    def cb(self, msg, var):

        self.blackboard[var] = msg

        now = rospy.get_time()

        # Ticks the root periodically to preserve the tick rate.
        if (now - self.last_tick_time) >= self.rate:

            self.tick_root()

            self.last_tick_time = now
        
    
