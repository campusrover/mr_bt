
#!/usr/bin/env python3

import rospy
from interpreter import TreeBuilder
from ros_behavior_tree import ROSBehaviorTree



import os

if __name__ == '__main__':

    print("hello")
    path = os.path.dirname(os.path.abspath(__file__)) + "/tree_jsons/"
    treefile = rospy.get_param("treefile")

    if treefile == "__NONE__":
        rospy.signal_shutdown("Check filepath for json file")

    tree_filepath = path + treefile

    rospy.init_node('btree')

    tb = TreeBuilder(tree_filepath)
    root, blackboard = tb.build_tree()
    # tb.draw_tree()
        

    # tree = ROSBehaviorTree(root, blackboard)
    # rospy.spin()
