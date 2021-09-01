#!/usr/bin/env python3

import rospy
from interpreter import TreeBuilder
from ros_behavior_tree import ROSBehaviorTree

if __name__ == '__main__':


    # rospy.init_node('person_follower')

    trees = ['ifelse']

    for name in trees:
        tb = TreeBuilder('tree_jsons/item_follower/item_follower.json')
        # root, blackboard = tb.build_tree()
        tb.draw_tree()
        # # root.tick(blackboard)
        # # print(name + ": " + str(root.tick(blackboard)))

    # tree = ROSBehaviorTree(node, blackboard, print_vars=["goal_pos","linear_pid","angular_pid", "avg_front_dist", "nearest_dist", "nearest_wall_angle", "position", "rotation", "goal_rotation", "dist"])
    # rospy.spin()

    