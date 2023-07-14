#!/usr/bin/env python3

import rospy
from interpreter import TreeBuilder
from ros_behavior_tree import ROSBehaviorTree

if __name__ == '__main__':

    tree = rospy.get_param("tree")
    log = rospy.get_param("log")

    rospy.init_node('btree')

    tb = TreeBuilder(tree)
    root, blackboard = tb.build_tree()        

    tree = ROSBehaviorTree(root, blackboard, log)
    rospy.spin()
