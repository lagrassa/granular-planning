import numpy as np
import planning.astar as astar

from envs.block_env import Simulator
from forward.state import EnvRepresentation 
from forward.node import Node, Graph

from forward.transition_models import *


# Set up the simulator
world = Simulator(gui=False, num_boxes=2)

robot_state  = np.array([0,0,0.05])
box_states  = [0.2,0.2,0,0.3]
state = np.hstack([robot_state,box_states])
world.set_state(state)
obs = world.get_state()
assert(np.allclose(state, obs))
shift_y = np.array([0,0.05,0])
world.apply_action(shift_y)
world.apply_action(shift_y)
world.apply_action(shift_y)
world.apply_action(shift_y)
print("Test passed")

# Create graph
robotState, blockStates = world.get_state()

g = Graph(np.array([[3, 3], [4, 4]]))
g.addVertex(robotState, blockStates, -1)

# check convex hull
ch = g.vertices[0].convexHull()
print(ch)

# check distance
g.computeHeuristics('8n')
print(g.vertices[0].h)

# A star
astar()

