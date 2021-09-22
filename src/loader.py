import importlib, os, inspect

from os import listdir, isfile

class Loader:


    def __init__(plugins=False):

        self.dirs = ['./nodes/update_nodes', './nodes/action_nodes', './nodes/conditional_nodes']

        self.dirs.append('.nodes/plugins') if plugins
        

    def list_files(self):

        
        files = [f for f in listdir()]


for name, cls in inspect.getmembers(importlib.import)