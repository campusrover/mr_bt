#!/usr/bin/env python3

import rospy
from interpreter import TreeBuilder
from ros_behavior_tree import ROSBehaviorTree

if __name__ == '__main__':


    rospy.init_node('person_follower')


    tb = TreeBuilder('tree_jsons/item_follower/item_follower.json')
    root, blackboard = tb.build_tree()
    tb.draw_tree()
        # # root.tick(blackboard)
        # # print(name + ": " + str(root.tick(blackboard)))

    tree = ROSBehaviorTree(root, blackboard)
    rospy.spin()

    