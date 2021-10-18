#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import numpy as np
import pydot

from nodes.nodes.node import Node
from nodes.nodes.parent_node import ParentNode


default_status_color_map = {"success":"green", "failure":"red", "running":"yellow"}


def build_dot(root: Node) -> pydot.Dot:
    graph = pydot.Dot("Behavior Tree", graph_type="digraph", bgcolor="white")
    recursive_init(graph, root)
    return graph


def recursive_init(graph: pydot.Dot, node: Node) -> None:

    dot_node = pydot.Node(node.id, label=node.name, color="gray")

    graph.add_node(dot_node)

    if issubclass(type(node), ParentNode):
        for child in node.children:
            recursive_init(graph, child)
            edge = pydot.Edge(node.id, child.id, color="black")
            graph.add_edge(edge)


def color_graph(graph: pydot.Dot, status_dict: dict, status_color_map: dict = default_status_color_map) -> pydot.Dot:

    executed = [*status_dict] # Returns list of node ids which have been executed
    
    for dot_node in graph.get_node_list():

        node_color = "gray"
        id = dot_node.get_name()
        if id in executed:
            node_color = status_color_map[status_dict[id]]

        dot_node.set_color(node_color)

    for edge in graph.get_edge_list():
        src = edge.get_source()
        dest = edge.get_destination()
        if (src in executed) and (dest in executed):
            edge.set_style("")
        else:
            edge.set_style("dotted")
    
    return graph


def graph_imgmsg_from_str(graph_str: str) -> Image:

        graph = pydot.graph_from_dot_data(graph_str)[0]

        byte_img = graph.create_jpg()

        np_img = np.frombuffer(byte_img, dtype=np.int8)

        cv_img = cv2.imdecode(np_img, cv2.IMREAD_UNCHANGED)

        return CvBridge().cv2_to_imgmsg(cv_img)


def same_tree_state(status_dict: dict, prev_status_dict: dict) -> bool:

    old_keys = list(prev_status_dict.keys())
    new_keys = list(status_dict.keys())

    if len(old_keys) != len(new_keys):
        return False

    for old_key, new_key in zip(old_keys, new_keys):
        if old_key != new_key:
            return False
        if prev_status_dict[old_key] != status_dict[new_key]:
            return False
    
    return True





# class Grapher:

#     def __init__(self, root: Node, status_color_map: dict = default_status_color_map):

#         self.root = root
#         self.status_color_map = status_color_map

#         self.prev_status_dict = {}
#         self.bridge = CvBridge()
#         self.img_pub = rospy.Publisher("/behavior_tree", Image, queue_size=10)

#         self.graph = pydot.Dot("Behavior Tree", graph_type="digraph", bgcolor="white")
#         self.recursive_init(self.root)


    
#     def recursive_init(self, node: Node) -> None:

#         dot_node = pydot.Node(node.id, label=node.name, color="gray")

#         self.graph.add_node(dot_node)

#         if issubclass(type(node), ParentNode):
#             for child in node.children:
#                 self.recursive_init(child)
#                 edge = pydot.Edge(node.id, child.id, color="black")
#                 self.graph.add_edge(edge)


#     def color_graph(self, status_dict: dict) -> None:

#         executed = [*status_dict] # Returns list of node ids which have been executed
        
#         for dot_node in self.graph.get_node_list():

#             node_color = "gray"
#             id = dot_node.get_name()
#             if id in executed:
#                 node_color = self.status_color_map[status_dict[id]]

#             dot_node.set_color(node_color)

#         for edge in self.graph.get_edge_list():
#             src = edge.get_source()
#             dest = edge.get_destination()
#             if (src in executed) and (dest in executed):
#                 edge.set_style("")
#             else:
#                 edge.set_style("dotted")

    
#     def publish_graph_img(self, status_dict: dict) ->None:



#         if self.same_tree_state(status_dict):
#             return 
        
#         self.prev_status_dict = status_dict

#         self.color_graph(status_dict)

#         byte_img = self.graph.create_jpg()

#         np_img = np.frombuffer(byte_img, dtype=np.int8)

#         cv_img = cv2.imdecode(np_img, cv2.IMREAD_UNCHANGED)

#         img_msg = self.bridge.cv2_to_imgmsg(cv_img)

#         self.img_pub.publish(img_msg)
    

#     def same_tree_state(status_dict: dict, prev_status_dict: dict) -> bool:

#         old_keys = list(prev_status_dict.keys())
#         new_keys = list(status_dict.keys())

#         if len(old_keys) != len(new_keys):
#             return False

#         for old_key, new_key in zip(old_keys, new_keys):
#             if old_key != new_key:
#                 return False
#             if prev_status_dict[old_key] != status_dict[new_key]:
#                 return False
        
#         return True


# # For testing, delete later
# # if __name__ == "__main__":

