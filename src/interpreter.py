#!/usr/bin/env python3

'''

RULES FOR THE JSON FORMATTING:

    NODES:

        For each node in a tree, you must provide a "name" parameter and a "type" parameter.

            The "name" field is a string that will be displayed in the Graphviz tree along with the "type" for each node. Each node in a tree must have
            a unique "name" in order for the tree to be displayed properly, but other than that the "name" of a node is arbitrary.

            The "type" parameter is used to specify which type of node you are instantiating. You must use one of the currently available
            node types listed above in the master_node_dict.

        When you are declaring a parent node, you will have a "children" parameter that will ask for a list of other nodes. You must provide
        a list of newly specified nodes in the same format as you would provide information for a regular node. This will give your .json file
        a nested structure.

    REFERENCES:

        You may pass in a reference to another json file node/tree structure as a child of another node. To do this, when declaring the node you must
        pass in an argument "ref" and assign it to the path of the referenced file relative to the interpreter.

    BLACKBOARD:

        You will need to provide a blackboard with the necessary variables to keep track of inside of your .json file. You will put this blackboard in
        as a parameter of the parent node and name it "blackboard".

        There are two types of blackboard variables that can be used in the blackboard.

            The "generic" variables which can be any kind of object or primitive data type supported by python. These types can have any name. They can be
            specified to initially have a null value, or start with a value of a data type supported by json.

            The ROS message variables will have the names of the topic which they are published to. Their name must start with a "/" or they will not be recognized
            as a ROS message and a subscriber will not be instantiated for them. They must initially have a value which is a key for one of the ROS message types specified
            above in the master_msg_dict.

    EXAMPLE:

        {
            "name":"parent",
            "type":"Selector",
            "children":[
                {
                    "name":"child1",
                    "type":"SomeConditionalNode",
                    "some_param1":"foo"
                },
                {
                    "name":"child2",
                    "type":"SomeActionNode",
                    "random_param1":"bar"
                },
                {
                    "ref":"path/to/other/node.json"
                }
            ],
            "blackboard":{
                "/scan":"LaserScan",
                "some_var":null
            }
        }

'''

import rospy
import numpy as np
import json
import graphviz
import sys

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage

# from nodes.nodes import *

# from nodes.action_nodes.basic_movement import *

# from nodes.update_nodes.basic_updates import *
# from nodes.update_nodes.movement_control_updates import *
# from nodes.update_nodes.odom_updates import *
# from nodes.update_nodes.cv_updates import *
# from nodes.update_nodes.scan_updates import *

# from nodes.conditional_nodes.scan_conditionals import *
# from nodes.conditional_nodes.basic_conditionals import *
# from nodes.conditional_nodes.odom_conditionals import *

# from nodes.logic_nodes.logic_gate_nodes import *
# from nodes.logic_nodes.conditional_logic_nodes import *

from ros_behavior_tree import ROSBehaviorTree



from .loader import import_node


# master_node_dict = {
    
#     "Conditional":Conditional, "Action":Action, "Update":Update, "Sequencer":Sequencer, "Selector":Selector, "Multitasker":Multitasker,
#     "LinearStatic":LinearStatic, "LinearDynamic":LinearDynamic, "AngularStatic":AngularStatic, "AngularDynamic":AngularDynamic,
#     "LinearAngularStatic":LinearAngularStatic, "LinearAngularDynamic":LinearAngularDynamic, "Stop": Stop,
#     "FlipBoolVar":FlipBoolVar, "IncrementVar":IncrementVar, "OffsetVar":OffsetVar,
#     "LinearPID":LinearPID, "AngularPID":AngularPID,
#     "GetPosition":GetPosition, "GetRotation":GetRotation,
#     "FastDetector":FastDetector, "ItemBearingErr":ItemBearingErr,
#     "CalcNearestWallAngle":CalcNearestWallAngle, "CalcNearestDist":CalcNearestDist, "CalcAvgFrontDist":CalcAvgFrontDist,
#     "WallAhead":WallAhead, "ClearAhead":ClearAhead,
#     "BoolVar":BoolVar, "BoolVarNot":BoolVarNot
# }

# master_msg_dict = {

#     "Twist":Twist, "LaserScan":LaserScan, "CompressedImage":CompressedImage, "Odometry":Odometry, "Pose":Pose
# }

# no_no_dict = {
#     '__annotations__':None, '__builtins__':None, '__cached__':None, '__doc__':None, '__file__':None,
#     '__loader__':None,'__name__':None,'__package__':None,'__spec__':None,'graphviz':None, 'json':None,'np':None,
#     'rospy':None, 'sys':None, 'ROSBehaviorTree':None, 'TreeBuilder':None
# }




class TreeBuilder:


    def __init__(self, path, comment=""):

        with open(path) as f:
            self.tree_dict = json.load(f)

        self.dot = graphviz.Digraph(format='png', comment='Behavior Tree')

        self.blackboard = {}



    def build_tree(self):
        '''
        The recursive function attach_node() is called on the root of the tree, then the 
        ROS behavior tree root and the blackboard are returned.
        '''
        root = self.attach_node(self.tree_dict)

        return root, self.blackboard  


    def attach_node(self, node):

        parameters = []

        specials = ['name', 'type', 'blackboard']

        for parameter in node: # Each parameter provided in the json is interpreted and used to initialize the node

            if parameter == 'children': # Initializes all children recursively and appends them to a list which is then
                                        # passed as another parameter in the node
                children = []

                for child in node['children']:
                    if 'ref' in child: # Handles the case where the child is a reference to another json file
                        with open(child['ref']) as f:
                            child = json.load(f)
                    children.append(self.attach_node(child))
                
                parameters.append(children)
            elif parameter not in specials:
                
                parameters.append(node[parameter])

        if 'blackboard' in node: # If the blackboard is passed as a parameter its contents are added to the tree blackboard

            for var in node['blackboard']:
                
                if var[0] == '/':
                    self.blackboard[var] = eval(node['blackboard'][var])
                else:
                    self.blackboard[var] = node['blackboard'][var] 

        return import_node(node['type'])(*parameters)


    def draw_tree(self):
        '''
        The recursive function link_nodes() is called on the root of the tree, then the 
        the graph is drawn and a pdf is created using a Digraph object from GraphViz.
        '''

        root_name, blackboard = self.link_nodes(self.tree_dict)

        self.link_blackboard(root_name, blackboard)

        self.dot.render("tree_visuals/tree", view=True)

    
    def link_blackboard(self, root_name, blackboard):
        '''
        Links the blackboard defined by any of the nodes in the tree and attaches it
        to the passed node in the graph and displays its contents.
        '''
        
        blackboard_string = 'BLACKBOARD\n\n'
        for key in blackboard:
            blackboard_string += key + '  :  ' + str(blackboard[key]) + '\n'
        self.dot.node('Blackboard', blackboard_string, shape='rectangle')
        self.dot.edge('Blackboard', root_name)


    def link_nodes(self, node, parent_label=None, blackboard={}):

        label_string = node['name'] + "\ntype: " + node['type']

        node_label = node['name']

        if node['type'] == 'Selector': # Changes the shape of the node depending on the type
            shape = "box"
        elif node['type'] == "Sequencer":
            shape = "cds"
        else:
            shape = "oval"

        self.dot.node(node_label, label_string, shape=shape)

        if 'blackboard' in node: # Blackboard is visualized as being passed into the node it is initialized in
            for var in node['blackboard']:
                blackboard[var] = node['blackboard'][var]

        if 'children' in node: # Recursively creates all of the Graphviz children nodes

            for child in node['children']:

                if 'ref' in child:
                    with open(child['ref']) as f:
                        child = json.load(f)

                child_label, blackboard = self.link_nodes(child, parent_label=node_label, blackboard=blackboard)

                self.dot.edge(node_label, child_label)

        

        return node_label, blackboard



