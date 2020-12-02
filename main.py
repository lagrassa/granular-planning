import numpy as np
import planning.astar as astar
import ipdb

from envs.block_env import Simulator
from forward.state import EnvRepresentation 
from forward.node import Node, Graph

from forward.transition_models import *


def quantRobotState(robotState, t, xy_step, theta_step):
    qs = np.zeros(4)
    qs[1:3] //= xy_step
    qs[3] //= theta_step
    return qs.astype(np.int)

def quantBlockStates(blockStates, step):
    return (blockStates // step).astype(np.int)


# Set up the simulator
# goal size divided by cell size must be an integer
workspace_size = 15
goal_size = 0.3
num_cell_x = 100
num_cell_theta = 4

world = Simulator(workspace_size, goal_size, gui=False, num_boxes = 2)
robot_state  = [0.0,-0.37,0.05]
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
print("Simulator created")

# Create graph
quant_xy_step = workspace_size / num_cell_x
quant_theta_step = np.pi / num_cell_theta

robotState, blockStates = world.get_robot_blk_states()

rState = quantRobotState(robotState, 0, quant_xy_step, quant_theta_step)
bStates = quantBlockStates(blockStates, quant_xy_step)

numGoalCells = int(goal_size // quant_xy_step)
goalX, goalY = np.meshgrid(np.arange(numGoalCells), np.arange(numGoalCells))
goal = np.stack([goalX.flat, goalY.flat], axis=1)
g = Graph(goal, world)
g.addVertex(robotState, blockStates, -1)

# check convex hull
# ch = g.vertices[0].convexHull()
# print(ch)

# check distance
g.computeHeuristics('8n')
print("Graph created")

# A star
plan = astar.A_star(g)
print("Plan created")

