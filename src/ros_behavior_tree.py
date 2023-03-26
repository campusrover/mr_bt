#!/usr/bin/env python3

import rospy
import time
from graph_utils import build_dot, color_graph, same_tree_state

from std_msgs.msg import Byte, String
from sensor_msgs.msg import Image

import threading 




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

        self.curr_tick = 1

        self.print_vars = print_vars

        self.graph = build_dot(root)
        self.dot_pub = rospy.Publisher("graph_dot", String, queue_size=10)
        self.prev_status_dict = {}

        self.root = root
        self.blackboard = blackboard

        subscribers = []

        for var in blackboard:

            # Creates a new subscriber for each topic specified in the blackboard
            if var[0] == "/":
                
                subscribers.append(rospy.Subscriber(var, blackboard[var], self.cb, var))

                self.blackboard[var] = None

        self.tick_sub = rospy.Subscriber('/tick', Byte, self.tick_root)


    def tick_root(self, msg):

        status, status_dict = self.root.tick(self.blackboard)

        status_dict = {status: self.root.id, **status_dict}

        self.publish_dot_msg(status_dict)

        self.curr_tick += 1


    def cb(self, msg, var):

        self.blackboard[var] = msg



    def publish_dot_msg(self, status_dict: dict):

        # if not same_tree_state(status_dict, self.prev_status_dict):
        self.graph = color_graph(self.graph, status_dict)
        self.graph.to_string()
        dot_msg = String(self.graph.to_string())
        self.dot_pub.publish(dot_msg)
        self.prev_status_dict = status_dict
            
            
    
