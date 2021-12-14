# Mini-Rover Behavior Tree (MRBT)
The goal behind the MRBT project was to create a efficient, modular, and user-friendly solution for programming complex behaviors into the Turtlebot3 robot stack. Our
solution came in the form of a popular method in the videogaming industry for programming behaviors into NPCs (Non-Playable Characters in videogames; think enemies in HALO or Bioshock).
With the use of behavior trees, we are now able to program complex behaviors using a simple tree definition in a JSON file, and we can partition a behavior into multiple different
subtrees which can then be used elsewhere or standalone.

## Key Components of the Behavior Tree

Any complex behavior tree can be broken down into a few key components which highlight the overall logical structure of how they operate.

### Nodes
A behavior tree is a tree-shaped data structure consisting of nodes, each of which contain a logical method which executes code. The behavor tree is evaluated recursively starting 
at the root node. Each node has the abilility to execute code which will either run a script, or execute all of its children. Each node will also return one of 3 outputs to its 
parent: "success", "failure", or "running". There are two main types of nodes: the control-flow (parent nodes) nodes and the leaf nodes. The control flow nodes 

#### Control-Flow Nodes

- Selector
  - The Selector executes its children sequentially from left to right. 
  - If one of its children returns either "success" or "running", it will halt execution of its children and it will return the result of the child it stopped on.
  - If all of its children return "failure", the Selector will also return "failure".
- Sequencer
  - The Sequencer executes its children sequentially from left to right.
  - The Sequencer will not halt execution of its children unless one of them returns "failure" or "running", in which case it will also return "failure" or "running".
  - If all children return "success" the Sequencer will return "success"
