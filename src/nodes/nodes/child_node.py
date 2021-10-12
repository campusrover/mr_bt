from abc import abstractmethod
from .node import Node


class ChildNode(Node):

    def __init__(self):
        super(ChildNode, self).__init__()

    
    @abstractmethod
    def tick(self, blackboard:dict) -> tuple([str, dict]):
        return