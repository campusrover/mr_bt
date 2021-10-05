from pathlib import Path
import inspect
import importlib

def import_node(nodename):

    node_filepath = str(list(Path("nodes").rglob(nodename + ".py"))[0])

    node_module_name = node_filepath.replace("/", ".").replace(".py", "")

    node_module = importlib.import_module(node_module_name)

    node_class = inspect.getmembers(node_module, inspect.isclass)[0][1]

    return node_class



