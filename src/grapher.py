# !/usr/bin/env python3

import cv2
import numpy as np
from graphviz import Digraph
from nodes.nodes.node import Node
from nodes.nodes.parent_node import ParentNode


default_status_color_map = {"success":"green", "failure":"red", "running":"yellow"}

class Grapher:

    def __init__(self, root: Node, status_color_map: dict = default_status_color_map):

        self.root = root
        self.status_color_map = status_color_map


    
    def show(self, status_dict: dict):

        graph = Digraph()

        _ = self.recursive_build(graph, self.root, status_dict)

        byte_img = graph.pipe(format="png")

        np_img = np.frombuffer(byte_img, dtype=np.int8)

        cv_img = cv2.imdecode(np_img, cv2.IMREAD_UNCHANGED)

        cv2.imshow("Graph", cv_img)

        cv2.waitKey(1)


    def recursive_build(self, graph: Digraph, node: Node, status_dict: dict) -> str:

        id, name = node.id, node.name
        executed = [*status_dict] # Returns list of node ids which have been executed
        
        node_color = "grey"
        if id in executed:
            node_color = self.status_color_map[status_dict[id]]

        graph.attr("node", color=node_color)
        graph.node(id, name)

        if  issubclass(type(node), ParentNode):
            for child in node.children:
                edge_color = self.recursive_build(graph, child, status_dict)
                    
                graph.attr("edge", color = edge_color)
                graph.edge(id, child.id)

        return node_color



